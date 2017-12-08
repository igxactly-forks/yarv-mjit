/**********************************************************************

  mjit.c - MRI method JIT compiler infrastructure

  Copyright (C) 2017 Vladimir Makarov <vmakarov@redhat.com>.

**********************************************************************/

/* We utilize widely used C compilers (GCC and LLVM Clang) to
   implement MJIT.  We feed them a C code generated from ISEQ.  The
   industrial C compilers are slower than regular JIT engines.
   Generated code performance of the used C compilers has a higher
   priority over the compilation speed.

   So our major goal is to minimize the ISEQ compilation time when we
   use widely optimization level (-O2).  It is achieved by

   o Using a precompiled version of the header
   o Keeping all files in `/tmp`.  On modern Linux `/tmp` is a file
     system in memory. So it is pretty fast
   o Implementing MJIT as a multi-threaded code because we want to
     compile ISEQs in parallel with iseq execution to speed up Ruby
     code execution.  MJIT has one thread (*worker*) to do
     parallel compilations:
      o It prepares a precompiled code of the minimized header.
	It starts at the MRI execution start
      o It generates PIC object files of ISEQs
      o It takes one JIT unit from a priority queue unless it is empty.
      o It translates the JIT unit ISEQ into C-code using the precompiled
        header, calls CC and load PIC code when it is ready
      o Currently MJIT put ISEQ in the queue when ISEQ is called
      o MJIT can reorder ISEQs in the queue if some ISEQ has been called
        many times and its compilation did not start yet
      o MRI reuses the machine code if it already exists for ISEQ
      o The machine code we generate can stop and switch to the ISEQ
        interpretation if some condition is not satisfied as the machine
        code can be speculative or some exception raises
      o Speculative machine code can be canceled.

   Here is a diagram showing the MJIT organization:

                 _______
                |header |
                |_______|
                    |                         MRI building
      --------------|----------------------------------------
                    |                         MRI execution
     	            |
       _____________|_____
      |             |     |
      |          ___V__   |  CC      ____________________
      |         |      |----------->| precompiled header |
      |         |      |  |         |____________________|
      |         |      |  |              |
      |         | MJIT |  |              |
      |         |      |  |              |
      |         |      |  |          ____V___  CC  __________
      |         |______|----------->| C code |--->| .so file |
      |                   |         |________|    |__________|
      |                   |                              |
      |                   |                              |
      | MRI machine code  |<-----------------------------
      |___________________|             loading


   We don't use SIGCHLD signal and WNOHANG waitpid in MJIT as it
   might mess with ruby code dealing with signals.  Also as SIGCHLD
   signal can be delivered to non-main thread, the stack might have a
   constraint.  So the correct version of code based on SIGCHLD and
   WNOHANG waitpid would be very complicated.  */

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#else
#include <sys/wait.h>
#include <sys/time.h>
#include <dlfcn.h>
#endif

#include "vm_core.h"
#include "mjit.h"
#include "version.h"

extern void native_mutex_lock(rb_nativethread_lock_t *lock);
extern void native_mutex_unlock(rb_nativethread_lock_t *lock);
extern void native_mutex_initialize(rb_nativethread_lock_t *lock);
extern void native_mutex_destroy(rb_nativethread_lock_t *lock);

extern void native_cond_initialize(rb_nativethread_cond_t *cond, int flags);
extern void native_cond_destroy(rb_nativethread_cond_t *cond);
extern void native_cond_signal(rb_nativethread_cond_t *cond);
extern void native_cond_broadcast(rb_nativethread_cond_t *cond);
extern void native_cond_wait(rb_nativethread_cond_t *cond, rb_nativethread_lock_t *mutex);

extern int rb_thread_create_mjit_thread(void (*child_hook)(void), void (*worker_func)(void));

#define RB_CONDATTR_CLOCK_MONOTONIC 1

#ifdef _WIN32
#define dlopen(name,flag) ((void*)LoadLibrary(name))
#define dlerror() strerror(rb_w32_map_errno(GetLastError()))
#define dlsym(handle,name) ((void*)GetProcAddress((handle),(name)))
#define dlclose(handle) (CloseHandle(handle))
#define RTLD_NOW  -1

#define waitpid(pid,stat_loc,options) (WaitForSingleObject((pid), INFINITE), GetExitCodeProcess((pid), (stat_loc)))
#define WIFEXITED(S) ((S) != STILL_ACTIVE)
#define WEXITSTATUS(S) (S)
#define WIFSIGNALED(S) (0)
typedef intptr_t pid_t;
#endif

/* Return time in milliseconds as a double.  */
static double
real_ms_time(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return tv.tv_usec / 1000.0 + tv.tv_sec * 1000.0;
}

/* A copy of MJIT portion of MRI options since MJIT initialization.  We
   need them as MJIT threads still can work when the most MRI data were
   freed. */
struct mjit_options mjit_opts;

/* Default level of details in the debug info.  */
static int debug_level = 3;

/* The MJIT start time.  */
static double mjit_time_start;

/* Return time in milliseconds as a double relative to MJIT start.  */
static double
relative_ms_time(void)
{
    return real_ms_time() - mjit_time_start;
}

/* TRUE if MJIT is initialized and will be used.  */
int mjit_init_p = FALSE;


/*  A compilation unit is called a JIT unit.  Only one ISEQ can belong
    to a unit.  The unit has a status described by the following
    enumeration.

    The diagram shows the possible status transitions:

    NOT_FORMED -> IN_QUEUE -> IN_GENERATION -> FAILED
                     ^             |            ^
                     |             |            |   load_unit
                     |             |            |
                     |              -------> SUCCESS ----> LOADED
                     |                          |            |
                     |                          V            |
                      ------------------<--------------------
                                change in global speculation
*/

enum unit_status {
    /* The unit is not in the queue.  An ISEQ can be added to it.  */
    UNIT_NOT_FORMED,
    /* The unit is in the queue for compilation.  */
    UNIT_IN_QUEUE,
    /* The unit is being processed by a MJIT worker (C code
       generation, the C code compilation, and the object code
       load). */
    UNIT_IN_GENERATION,
    /* Unit compilation or its load failed.  */
    UNIT_FAILED,
    /* Unit compilation successfully finished.  */
    UNIT_SUCCESS,
    /* The unit ISEQ machine code was successfully loaded.  */
    UNIT_LOADED,
};

/* The unit structure.  */
struct rb_mjit_unit {
    int num; /* unit order number */
    enum unit_status status;
    /* Name of the C code file of the unit ISEQ.  Defined for status
       UNIT_IN_GENERATION.  */
    char *cfname;
    /* Name of the object file of the unit ISEQ.  Defined for status
       UNIT_IN_GENERATION and UNIT_SUCCESS.  */
    char *ofname;
    /* PID of C compiler processing the unit.  Defined for status
       UNIT_IN_GENERATION. */
    pid_t pid;
    /* Units in lists are linked with the following members.  */
    struct rb_mjit_unit *next, *prev;
    /* Dlopen handle of the loaded object file.  Defined for status
       UNIT_LOADED.  */
    void *handle;
    /* ISEQ in the unit.  We have at most one unit iseq.  */
    struct rb_mjit_unit_iseq *unit_iseq;
    /* If the flag is TRUE, the corresponding iseq was freed during
       compilation.  */
    char freed_p;
    /* Overall byte code size of ISEQ in the unit in VALUEs.  */
    size_t iseq_size;
    /* The relative time when a worker started to process the unit.
       It is used in the debug mode.  */
    double time_start;
};

/* The structure describing an ISEQ in the unit.  We create at most
   one such structure for a particular iseq.  */
struct rb_mjit_unit_iseq {
    /* Unique order number of unit ISEQ.  */
    int num;
    rb_iseq_t *iseq;
    /* The ISEQ byte code size in VALUEs.  */
    size_t iseq_size;
    struct rb_mjit_unit *unit;
    /* All unit iseqs are chained by the following field.  */
    struct rb_mjit_unit_iseq *next;
    /* The fields used for profiling only:  */
    char *label; /* Name of the ISEQ */
    char used_code_p;
    /* See the corresponding fields in iseq_constant_body.  The values
       are saved for GCed iseqs.  */
    unsigned long resume_calls, stop_calls;
};

/* Defined in the client thread before starting MJIT threads:  */
/* Used C compiler path.  */
static char *cc_path;
/* Name of the header file.  */
static const char *header_fname;
/* Name of the precompiled header file.  */
static const char *pch_fname;
/* Windows-supported /tmp */
static const char *tmp_dirname;

/* Return length of NULL-terminated array ARGS excluding the NULL
   marker.  */
static size_t
args_len(char *const *args)
{
    size_t i;

    for (i = 0; (args[i]) != NULL;i++)
	;
    return i;
}

/* Concatenate NUM passed NULL-terminated arrays of strings, put the
   result (with NULL end marker) into the heap, and return the
   result.  */
static char **
form_args(int num, ...)
{
    va_list argp, argp2;
    size_t len, disp;
    int i;
    char **args, **res;

    va_start(argp, num);
    va_copy(argp2, argp);
    for (i = len = 0; i < num; i++) {
	args = va_arg(argp, char **);
	len += args_len(args);
    }
    va_end(argp);
    if ((res = xmalloc((len + 1) * sizeof(char *))) == NULL)
	return NULL;
    for (i = disp = 0; i < num; i++) {
	args = va_arg(argp2, char **);
	len = args_len(args);
	memmove(res + disp, args, len * sizeof(char *));
	disp += len;
    }
    res[disp] = NULL;
    va_end(argp2);
    return res;
}

/* Make and return copy of STR in the heap.  Return NULL in case of a
   failure.  */
static char *
get_string(const char *str)
{
    char *res;

    if ((res = xmalloc(strlen(str) + 1)) != NULL)
	strcpy(res, str);
    return res;
}

/* Return an unique file name in /tmp with PREFIX and SUFFIX and
   number ID.  Use getpid if ID == 0.  The return file name exists
   until the next function call.  */
static char *
get_uniq_fname(unsigned long id, const char *prefix, const char *suffix)
{
    char str[70];
    sprintf(str, "%s/%sp%luu%lu%s", tmp_dirname, prefix, (unsigned long) getpid(), id, suffix);
    return get_string(str);
}

/* Maximum length for C function name generated for an ISEQ.  */
#define MAX_MJIT_FNAME_LEN 30

/* Put C function name of unit iseq UI into HOLDER.  Return
   HOLDER.  */
static char *
get_unit_iseq_fname(struct rb_mjit_unit_iseq *ui, char *holder)
{
    sprintf(holder, "_%d", ui->num);
    return holder;
}

/* A mutex for conitionals and critical sections.  */
static rb_nativethread_lock_t mjit_engine_mutex;
/* A thread conditional to wake up workers if at the end of PCH thread.  */
static rb_nativethread_cond_t mjit_pch_wakeup;
/* A thread conditional to wake up the client if there is a change in
   executed unit status.  */
static rb_nativethread_cond_t mjit_client_wakeup;
/* A thread conditional to wake up a worker if there we have something
   to add or we need to stop MJIT engine.  */
static rb_nativethread_cond_t mjit_worker_wakeup;
/* A thread conditional to wake up workers if at the end of GC.  */
static rb_nativethread_cond_t mjit_gc_wakeup;
/* Stop generation of a JITted code.  */
static int stop_mjit_generation_p;
/* True when GC is working.  */
static int in_gc;
/* Number of units being currently translated.  */
static int units_in_translation;

/* Doubly linked list of units.  */
struct rb_mjit_unit_list {
    struct rb_mjit_unit *head, *tail;
    int length; /* the list length */
};

/* The unit queue.  The client and MJIT threads work on the queue.
   So code using the following variable should be synced.  */
static struct rb_mjit_unit_list unit_queue;

/* The client and MJIT threads work on the list of active units
   (doubly linked list).  So code using the following variable should
   be synced.  All units with status UNIT_LOADED are in this list.
   They have also non-null handles.  */
static struct rb_mjit_unit_list active_units;

/* The client and MJIT threads work on the list of obsolete units
   (doubly linked list).  So code using the following variable should
   be synced.  All units in this list have status UNIT_FAILED.  They
   have also non-null handles.  */
static struct rb_mjit_unit_list obsolete_units;

/* The following functions are low level (ignoring thread
   synchronization) functions working with the lists.  */

/* Make non-zero to check consistency of MJIT lists.  */
#define MJIT_CHECK_LISTS 0

#if MJIT_CHECK_LISTS

/* Check double linked LIST consistency.  */
static void
check_list(struct rb_mjit_unit_list *list)
{
    if (list->head == NULL)
	assert(list->tail == NULL);
    else {
	struct rb_mjit_unit *u, *pu;

	assert(list->tail != NULL);
	assert(list->head->prev == NULL);
	assert(list->tail->next == NULL);
	for (pu = NULL, u = list->head; u != NULL; pu = u, u = u->next) {
	    assert(u->prev == pu);
	}
	for (u = NULL, pu = list->tail; pu != NULL; u = pu, pu = pu->prev) {
	    assert(pu->next == u);
	}
    }
}

#endif

/* Initiate LIST.  */
static void
init_list(struct rb_mjit_unit_list *list)
{
    list->tail = list->head = NULL;
    list->length = 0;
}

/* Add unit U to the tail of doubly linked LIST.  It should be not in
   the list before.  */
static void
add_to_list(struct rb_mjit_unit *u, struct rb_mjit_unit_list *list)
{
    u->next = u->prev = NULL;
    if (list->head == NULL)
	list->head = list->tail = u;
    else {
	list->tail->next = u;
	u->prev = list->tail;
	list->tail = u;
    }
    list->length++;
#if MJIT_CHECK_LISTS
    check_list(list);
#endif
}

/* Remove unit U from the doubly linked LIST.  It should be in the
   list before.  */
static void
remove_from_list(struct rb_mjit_unit *u, struct rb_mjit_unit_list *list)
{
    if (u == list->head)
	list->head = u->next;
    else
	u->prev->next = u->next;
    if (u == list->tail)
	list->tail = u->prev;
    else
	u->next->prev = u->prev;
    list->length--;
#if MJIT_CHECK_LISTS
    check_list(list);
#endif
}

/* Remove and return the best unit from doubly linked LIST.  The best
   is the first high priority unit or the unit whose iseq has the
   biggest number of calls so far.  */
static struct rb_mjit_unit *
get_from_list(struct rb_mjit_unit_list *list)
{
    struct rb_mjit_unit *u;
    struct rb_mjit_unit *best_u = NULL;
    struct rb_mjit_unit_iseq *ui;
    unsigned long calls_num, best_calls_num = 0;

#if MJIT_CHECK_LISTS
    check_list(list);
#endif
    for (u = list->head; u != NULL; u = u->next) {
	calls_num = 0;
	ui = u->unit_iseq;
	assert(ui != NULL);
	if (ui->iseq != NULL)
	    calls_num += ui->iseq->body->resume_calls + ui->iseq->body->stop_calls;
	if (best_u == NULL || calls_num > best_calls_num) {
	    best_u = u;
	    best_calls_num = calls_num;
	}
    }
    if (best_u != NULL)
	remove_from_list(best_u, list);
    return best_u;
}

/* Print the arguments according to FORMAT to stderr only if MJIT
   verbose option value is more or equal to LEVEL.  */
PRINTF_ARGS(static void, 2, 3)
verbose(int level, const char *format, ...)
{
    va_list args;

    va_start(args, format);
    if (mjit_opts.verbose >= level) {
	char str[256];
	vsprintf(str, format, args);
	fprintf(stderr, "+++%s: time - %.3f ms\n", str, relative_ms_time());
    }
    va_end(args);
}

/* Print the arguments according to FORMAT to stderr only if the
   message LEVEL is not greater to the current debug level.  */
PRINTF_ARGS(static void, 2, 3)
debug(int level, const char *format, ...)
{
    va_list args;

    if (! mjit_opts.debug || ! mjit_opts.verbose)
	return;
    va_start(args, format);
    if (debug_level >= level) {
	char str[256];
	vsprintf(str, format, args);
	fprintf(stderr, "+++%s: time - %.3f ms\n", str, relative_ms_time());
    }
    va_end(args);
}

/* Start a critical section.  Use message MSG to print debug info at
   LEVEL.  */
static inline void
CRITICAL_SECTION_START(int level, const char *msg)
{
    debug(level, "Locking %s", msg);
    native_mutex_lock(&mjit_engine_mutex);
    debug(level, "Locked %s", msg);
}

/* Finish the current critical section.  Use message MSG to print
   debug info at LEVEL. */
static inline void
CRITICAL_SECTION_FINISH(int level, const char *msg)
{
    debug(level, "Unlocked %s", msg);
    native_mutex_unlock(&mjit_engine_mutex);
}

/* XXX_COMMONN_ARGS define the command line arguments of XXX C
   compiler used by MJIT.

   XXX_USE_PCH_ARAGS define additional options to use the precomiled
   header.  */
static const char *GCC_COMMON_ARGS_DEBUG[] = {"gcc", "-O0", "-g", "-Wfatal-errors", "-fPIC", "-shared", "-w", "-pipe", "-nostartfiles", "-nodefaultlibs", "-nostdlib", NULL};
static const char *GCC_COMMON_ARGS[] = {"gcc", "-O2", "-Wfatal-errors", "-fPIC", "-shared", "-w", "-pipe", "-nostartfiles", "-nodefaultlibs", "-nostdlib", NULL};

#ifdef __MACH__

static const char *LLVM_COMMON_ARGS_DEBUG[] = {"clang", "-O0", "-g", "-dynamic", "-I/usr/local/include", "-L/usr/local/lib", "-w", "-bundle", NULL};
static const char *LLVM_COMMON_ARGS[] = {"clang", "-O2", "-dynamic", "-I/usr/local/include", "-L/usr/local/lib", "-w", "-bundle", NULL};

#else

static const char *LLVM_COMMON_ARGS_DEBUG[] = {"clang", "-O0", "-g", "-fPIC", "-shared", "-I/usr/local/include", "-L/usr/local/lib", "-w", "-bundle", NULL};
static const char *LLVM_COMMON_ARGS[] = {"clang", "-O2", "-fPIC", "-shared", "-I/usr/local/include", "-L/usr/local/lib", "-w", "-bundle", NULL};

#endif /* #ifdef __MACH__ */

static const char *LLVM_USE_PCH_ARGS[] = {"-include-pch", NULL, "-Wl,-undefined", "-Wl,dynamic_lookup", NULL};

/*-------All the following code is executed in the worker threads only-----------*/

/* Translate iseq of unit U into C code and output it to the
   corresponding file.  Add include directives with INCLUDE_FNAME
   unless it is NULL.  Return 0 for a success. Otherwise non-0 value */
static int
translate_unit_iseq(struct rb_mjit_unit *u, const char *include_fname)
{
    int fd, err, success_p;
    FILE *f = fopen(u->cfname, "w");
    char mjit_fname_holder[MAX_MJIT_FNAME_LEN];

    if (f == NULL)
	return errno;
    if (include_fname != NULL)
	fprintf(f, "#include \"%s\"\n", include_fname);

#ifdef _WIN32
    /* Requirements for DLL */
    fprintf(f, "void _pei386_runtime_relocator(void){}\n");
    fprintf(f, "int __stdcall DllMainCRTStartup(void* hinstDLL, unsigned int fdwReason, void* lpvReserved) { return 1; }\n");
#endif

    success_p = mjit_compile(f, u->unit_iseq->iseq->body, get_unit_iseq_fname(u->unit_iseq, mjit_fname_holder));

    fd = fileno(f);
    fsync(fd);
    err = ferror(f);
    fclose(f);
    return err || !success_p;
}

/* Start an OS process of executable PATH with arguments ARGV.  Return
   PID of the process.  */
static pid_t
start_process(const char *path, char *const *argv)
{
    pid_t pid;

    if (mjit_opts.verbose >= 2) {
	int i;
	const char *arg;

	fprintf(stderr, "++Starting process: %s", path);
	for (i = 0; (arg = argv[i]) != NULL; i++)
	    fprintf(stderr, " %s", arg);
	fprintf(stderr, ": time - %.3f ms\n", relative_ms_time());
    }
#ifdef _WIN32
    pid = spawnvp(_P_NOWAIT, path, argv);
#else
    if ((pid = vfork()) == 0) {
	if (mjit_opts.verbose) {
	    /* CC can be started in a thread using a file which has been
	    already removed while MJIT is finishing.  Discard the
	    messages about missing files.  */
	    FILE *f = fopen("/dev/null", "w");

	    dup2(fileno(f), STDERR_FILENO);
	    dup2(fileno(f), STDOUT_FILENO);
	}
	pid = execvp(path, argv); /* Pid will be negative on an error */
	/* Even if we successfully found CC to compile PCH we still can
	fail with loading the CC in very rare cases for some reasons.
	Stop the forked process in this case.  */
	debug(1, "Error in execvp: %s", path);
	_exit(1);
    }
#endif
    return pid;
}

/* This function is executed in a worker thread.  The function creates
   a C file for iseqs in the unit U and starts a C compiler to
   generate an object file of the C file.  Return TRUE in a success
   case.  */
static int
start_unit(struct rb_mjit_unit *u)
{
    int fail_p;
    pid_t pid;
    static const char *input[] = {NULL, NULL};
    static const char *output[] = {"-o",  NULL, NULL};
#ifdef _WIN32
    /* Link to ruby.dll.a, because Windows DLLs don't allow unresolved symbols. */
    static const char *libs[] = {"-L" LIBRUBY_LIBDIR, LIBRUBYARG_SHARED, "-lmsvcrt"};
#else
    static const char *libs[] = {NULL};
#endif
    char **args;

    verbose(3, "Starting unit %d compilation", u->num);
    if ((u->cfname = get_uniq_fname(u->num, "_mjit", ".c")) == NULL) {
	u->status = UNIT_FAILED;
	return FALSE;
    }
    if ((u->ofname = get_uniq_fname(u->num, "_mjit", ".so")) == NULL) {
	u->status = UNIT_FAILED;
	free(u->cfname); u->cfname = NULL;
	return FALSE;
    }
    if (mjit_opts.debug)
	u->time_start = real_ms_time();
    CRITICAL_SECTION_START(3, "in worker to wait GC finish");
    while (in_gc) {
	debug(3, "Waiting wakeup from GC");
	native_cond_wait(&mjit_gc_wakeup, &mjit_engine_mutex);
    }
    units_in_translation++;
    CRITICAL_SECTION_FINISH(3, "in worker to wait GC finish");
    fail_p = translate_unit_iseq(u, mjit_opts.llvm ? NULL : header_fname);
    CRITICAL_SECTION_START(3, "in worker to wakeup client for GC");
    units_in_translation--;
    debug(3, "Sending wakeup signal to client in a mjit-worker for GC");
    native_cond_signal(&mjit_client_wakeup);
    CRITICAL_SECTION_FINISH(3, "in worker to wakeup client for GC");

    if (fail_p) {
	pid = -1;
    } else {
	input[0] = u->cfname;
	output[1] = u->ofname;
	if (mjit_opts.llvm) {
	    LLVM_USE_PCH_ARGS[1] = pch_fname;
	    args = form_args(5, (mjit_opts.debug ? LLVM_COMMON_ARGS_DEBUG : LLVM_COMMON_ARGS),
			     LLVM_USE_PCH_ARGS, input, output, libs);
	} else {
	    args = form_args(4, (mjit_opts.debug ? GCC_COMMON_ARGS_DEBUG : GCC_COMMON_ARGS),
			     input, output, libs);
	}
	if (args == NULL)
	    pid = -1;
	else {
	    pid = start_process(cc_path, args);
	    free(args);
	}
    }
    if (pid < 0) {
        debug(1, "Failed starting unit %d compilation", u->num);
	u->status = UNIT_FAILED;
	if (! mjit_opts.save_temps) {
	    remove(u->cfname);
	    free(u->cfname); u->cfname = NULL;
	    remove(u->ofname);
	    free(u->ofname); u->ofname = NULL;
	}
	return FALSE;
    } else {
	debug(2, "Success in starting unit %d compilation", u->num);
	u->pid = pid;
	return TRUE;
    }
}

static void discard_unit(struct rb_mjit_unit *u);

/* The function should be called after successul creation of the
   object file for iseq of unit U.  The function loads the object
   file.  */
static void
load_unit(struct rb_mjit_unit *u)
{
    struct rb_mjit_unit_iseq *ui;
    void *addr;
    char mjit_fname_holder[MAX_MJIT_FNAME_LEN];
    const char *fname, *err_name;
    void *handle;

    assert(u->status == UNIT_SUCCESS);
    handle = dlopen(u->ofname, RTLD_NOW);
    if (mjit_opts.save_temps) {
	CRITICAL_SECTION_START(3, "in load_unit to setup MJIT code");
    } else {
	const char *ofname = u->ofname;

	if (ofname != NULL)
	    remove(ofname);
	CRITICAL_SECTION_START(3, "in load_unit to setup MJIT code");
	if (u->ofname != NULL)
	    free(u->ofname);
	u->ofname = NULL;
    }
    if (handle != NULL)
	verbose(1, "Success in loading code of unit %d", u->num);
    else if (mjit_opts.warnings || mjit_opts.verbose)
	fprintf(stderr, "MJIT warning: failure in loading code of unit %d(%s)\n", u->num, dlerror());
    ui = u->unit_iseq;
    if (ui->iseq == NULL) {
	/* Garbage collected */
	if (handle != NULL)
	    dlclose(handle);
    } else {
	assert(ui != NULL);
	u->handle = handle;
	if (handle == NULL) {
	    addr = (void *) NOT_ADDED_JIT_ISEQ_FUN;
	    discard_unit(u);
	} else {
	    fname = get_unit_iseq_fname(ui, mjit_fname_holder);
	    addr = dlsym(handle, fname);
	    if ((err_name = dlerror()) != NULL) {
		debug(0, "Failure (%s) in setting address of iseq %d(%s)", err_name, ui->num, fname);
		addr = (void *)NOT_ADDED_JIT_ISEQ_FUN;
		add_to_list(u, &obsolete_units);
		u->status = UNIT_FAILED;
	    } else {
		debug(2, "Success in setting address of iseq %d(%s)(%s) 0x%"PRIxVALUE,
		      ui->num, fname, ui->label, (VALUE)addr);
		add_to_list(u, &active_units);
		u->status = UNIT_LOADED;
	    }
	}
	/* Usage of jit_code might be not in a critical section.  */
	ATOMIC_SET(ui->iseq->body->jit_code, addr);
    }
    CRITICAL_SECTION_FINISH(3, "in load_unit to setup MJIT code");
}

/* Maximum number of worker threads.  As worker can process only one
   unit at a time, the number also represents the maximal number of C
   compiler processes started by MJIT and running at any given
   time.  */
#define MAX_WORKERS_NUM 100
/* The default number of worker threads.  */
#define DEFAULT_WORKERS_NUM 1

/* Set to TRUE to finish worker.  */
static int finish_worker_p;
/* Set to TRUE after worker is finished */
static int worker_finished_p;

/* The function implementing a worker. It is executed in a separate
   thread started by rb_thread_create_mjit_thread. */
static void
worker(void)
{
    CRITICAL_SECTION_START(3, "in worker to start the unit");
    for (;;) {
	int stat, exit_code;
	struct rb_mjit_unit *u;

	if (finish_worker_p) {
	    worker_finished_p = TRUE;
	    break;
	}
	u = get_from_list(&unit_queue);
	if (u != NULL)
	    u->status = UNIT_IN_GENERATION;
	CRITICAL_SECTION_FINISH(3, "in worker to start the unit");
	if (u != NULL) {
	    exit_code = -1;
	    if (start_unit(u)) {
		waitpid(u->pid, &stat, 0);
		if (WIFEXITED(stat)) {
		    exit_code = WEXITSTATUS(stat);
		}
	    }
	    CRITICAL_SECTION_START(3, "in worker to setup status");
	    u->status = exit_code != 0 ? UNIT_FAILED : UNIT_SUCCESS;
	    if (exit_code == 0)
		verbose(3, "Success in compilation of unit %d", u->num);
	    else if (mjit_opts.warnings || mjit_opts.verbose)
		fprintf(stderr, "MJIT warning: failure in compilation of unit %d\n", u->num);
	    CRITICAL_SECTION_FINISH(3, "in worker to setup status");
	    if (!mjit_opts.save_temps) {
		remove(u->cfname);
		free(u->cfname); u->cfname = NULL;
	    }
	    CRITICAL_SECTION_START(3, "in worker to check global speculation status");
	    if (exit_code != 0 || u->freed_p) {
		discard_unit(u);
	    } else {
		CRITICAL_SECTION_FINISH(3, "in worker to check global speculation status");
		debug(2, "Start loading unit %d", u->num);
		load_unit(u);
		CRITICAL_SECTION_START(3, "in worker for a worker wakeup");
	    }
	    debug(3, "Sending wakeup signal to client in a mjit-worker");
	    native_cond_signal(&mjit_client_wakeup);
	} else {
	    debug(3, "Waiting wakeup from client");
	    CRITICAL_SECTION_START(3, "in worker for a worker wakeup");
	    while (unit_queue.head == NULL && ! finish_worker_p) {
		native_cond_wait(&mjit_worker_wakeup, &mjit_engine_mutex);
		debug(3, "Getting wakeup from client");
	    }
	}
    }
    CRITICAL_SECTION_FINISH(3, "in worker to finish");
    debug(1, "Finishing worker");
}

/*-------All the following code is executed in the client thread only-----------*/
/* The current number of worker threads. */
static int workers_num;

/* Singly linked list of allocated but marked free unit structures.  */
static struct rb_mjit_unit *free_unit_list;
/* Singly linked list of all allocated but unit iseq structures.  */
static struct rb_mjit_unit_iseq *unit_iseq_list;
/* The number of so far processed ISEQs.  */
static int curr_unit_iseq_num;
/* The unit currently being formed.  */
static struct rb_mjit_unit *curr_unit;
/* The number of so far created units.  */
static int curr_unit_num;

/* Initialize code for work with workers.  */
static void
init_workers(void)
{
    workers_num = 0;
    free_unit_list = NULL;
    unit_iseq_list = NULL;
    curr_unit_iseq_num = 0;
    init_list(&unit_queue);
    init_list(&active_units);
    init_list(&obsolete_units);
    curr_unit = NULL;
    curr_unit_num = 0;
}

/* Return a free unit iseq.  It can allocate a new structure if there
   are no free unit iseqs.  */
static struct rb_mjit_unit_iseq *
create_unit_iseq(rb_iseq_t *iseq)
{
    struct rb_mjit_unit_iseq *ui;

    ui = xmalloc(sizeof(struct rb_mjit_unit_iseq));
    if (ui == NULL)
	return NULL;
    ui->num = curr_unit_iseq_num++;
    ui->iseq = iseq;
    iseq->body->unit_iseq = ui;
    ui->unit = NULL;
    ui->next = unit_iseq_list;
    unit_iseq_list = ui;
    ui->iseq_size = iseq->body->iseq_size;
    ui->label = NULL;
    if (mjit_opts.verbose >= 2 || mjit_opts.debug) {
	ui->label = get_string(RSTRING_PTR(iseq->body->location.label));
	ui->resume_calls = ui->stop_calls = 0;
    }
    ui->used_code_p = FALSE;
    return ui;
}

/* Return a free unit iseq.  */
static struct rb_mjit_unit *
create_unit(void)
{
    struct rb_mjit_unit *u;

    if (free_unit_list == NULL) {
	u = xmalloc(sizeof(struct rb_mjit_unit));
	if (u == NULL)
	    return NULL;
    } else {
	u = free_unit_list;
	free_unit_list = free_unit_list->next;
    }
    u->status = UNIT_NOT_FORMED;
    u->num = curr_unit_num++;
    u->iseq_size = 0;
    u->cfname = u->ofname = NULL;
    u->handle = NULL;
    u->freed_p = FALSE;
    u->next = NULL;
    u->unit_iseq = NULL;
    return u;
}

/* Mark the unit U free.  Unload the JITted code.  Remove the object
   file if it exists.  */
static void
free_unit(struct rb_mjit_unit *u)
{
    verbose(3, "Removing unit %d", u->num);
    if (u->handle != NULL) {
	dlclose(u->handle);
	u->handle = NULL;
    }
    if (u->status == UNIT_SUCCESS && ! mjit_opts.save_temps) {
	remove(u->ofname);
    }
    if (u->ofname != NULL) {
	free(u->ofname);
	u->ofname = NULL;
    }
    u->next = free_unit_list;
    free_unit_list = u;
}

/* Mark the unit from LIST as free.  */
static void
free_units(struct rb_mjit_unit *list)
{
    struct rb_mjit_unit *u, *next;

    for (u = list; u != NULL; u = next) {
	next = u->next;
	free_unit(u);
    }
}

/* Free memory for all marked free units and unit iseqs.  */
static void
finish_units(void)
{
    struct rb_mjit_unit *u, *next;
    struct rb_mjit_unit_iseq *ui, *ui_next;

    for (u = free_unit_list; u != NULL; u = next) {
	next = u->next;
	free(u);
    }
    for (ui = unit_iseq_list; ui != NULL; ui = ui_next) {
	ui_next = ui->next;
	free(ui);
    }
    free_unit_list = NULL;
    unit_iseq_list = NULL;
}

/* Free memory allocated for all units and unit iseqs.  */
static void
finish_workers(void)
{
    free_units(obsolete_units.head);
    free_units(active_units.head);
    free_units(unit_queue.head);
    if (curr_unit != NULL)
	free_unit(curr_unit);
    finish_units();
}

/* Add the unit iseq UI to the unit U.  */
static void
add_iseq_to_unit(struct rb_mjit_unit *u, struct rb_mjit_unit_iseq *ui)
{
    assert(u->unit_iseq == NULL);
    ui->unit = u;
    u->unit_iseq = ui;
    u->iseq_size += ui->iseq_size;
    debug(1, "iseq %d is added to unit %d (size %lu)", ui->num, u->num, (long unsigned)u->iseq_size);
}

static void
finish_forming_unit(struct rb_mjit_unit *u)
{
    assert(u != NULL);
    add_to_list(u, &unit_queue);
    u->status = UNIT_IN_QUEUE;
    debug(2, "Finish forming unit %d (size = %lu)", u->num, (long unsigned)u->iseq_size);
}

/* Add the current unit to the queue.  */
static void
finish_forming_curr_unit(void)
{
    if (curr_unit == NULL)
	return;
    finish_forming_unit(curr_unit);
    curr_unit = NULL;
}

/* Create unit and iseq unit for ISEQ.  Reuse the iseq unit if it
   already exists.  */
static void
create_iseq_unit(rb_iseq_t *iseq)
{
    struct rb_mjit_unit_iseq *ui;

    if (curr_unit == NULL && (curr_unit = create_unit()) == NULL)
	return;
    if ((ui = iseq->body->unit_iseq) == NULL
	&& (ui = create_unit_iseq(iseq)) == NULL)
	return;
    add_iseq_to_unit(curr_unit, ui);
}

/* Detach the iseq unit from unit U and free the unit.  */
static void
discard_unit(struct rb_mjit_unit *u)
{
    struct rb_mjit_unit_iseq *ui = u->unit_iseq;

    assert(ui->unit == u);
    ui->unit = NULL;
    free_unit(u);
}


/* MJIT info related to an existing continutaion.  */
struct mjit_cont {
    rb_execution_context_t *ec; /* continuation thread */
    struct mjit_cont *prev, *next; /* used to form lists */
};

/* Double linked list of registered continuations.  */
struct mjit_cont *first_cont;
/* List of unused mjit_cont structures.  */
struct mjit_cont *free_conts;

/* Initiate continuation info in MJIT.  */
static void
init_conts(void)
{
    free_conts = first_cont = NULL;
}

/* Create and return continuation info in MJIT.  Include it to the
   continuation list.  EC is the continuation ec.  */
static struct mjit_cont *
create_cont(rb_execution_context_t *ec)
{
    struct mjit_cont *cont;

    if (free_conts != NULL) {
	cont = free_conts;
	free_conts = free_conts->next;
    } else if ((cont = xmalloc(sizeof(struct mjit_cont))) == NULL) {
	/* We can not generate and use code without the continuation
	   info.  */
	stop_mjit_generation_p = TRUE;
	return NULL;
    }
    cont->ec = ec;
    if (first_cont == NULL) {
	cont->next = cont->prev = NULL;
    } else {
	cont->prev = NULL;
	cont->next = first_cont;
	first_cont->prev = cont;
    }
    first_cont = cont;
    return cont;
}

/* Remove continuation info CONT from the continuation list.  Include
   it into the free list.  */
static void
free_cont(struct mjit_cont *cont)
{
    if (cont == first_cont) {
	first_cont = cont->next;
	if (first_cont != NULL)
	    first_cont->prev = NULL;
    } else {
	cont->prev->next = cont->next;
	if (cont->next != NULL)
	    cont->next->prev = cont->prev;
    }
    cont->next = free_conts;
    free_conts = cont;
}

/* Finish work with continuation info (free all continuation
   structures).  */
static void
finish_conts(void)
{
    struct mjit_cont *cont, *next;

    for (cont = first_cont; cont != NULL; cont = next) {
	next = cont->next;
	free_cont(cont);
    }
    for (cont = free_conts; cont != NULL; cont = next) {
	next = cont->next;
	free(cont);
    }
}

/* Default permitted number of units with a JIT code kept in
   memory.  */
#define DEFAULT_CACHE_SIZE 1000
/* Minimum value for JIT cache size.  */
#define MIN_CACHE_SIZE 10

/* Clear used_code_p field for unit iseqs of units in LIST.  */
static void
mark_unit_iseqs(struct rb_mjit_unit_list *list)
{
    struct rb_mjit_unit *u;

    for (u = list->head; u != NULL; u = u->next) {
	assert(u->handle != NULL && u->unit_iseq != NULL);
	u->unit_iseq->used_code_p = FALSE;
    }
}

/* Set up field used_code_p for unit iseqs whose iseq on the stack of ec.  */
static void
mark_thread_unit_iseqs(rb_execution_context_t *ec)
{
    rb_iseq_t *iseq;
    const rb_control_frame_t *fp;
    struct rb_mjit_unit_iseq *ui;
    rb_control_frame_t *last_cfp = ec->cfp;
    const rb_control_frame_t *end_marker_cfp;
    ptrdiff_t i, size;

    if (ec->vm_stack == NULL)
	return;
    end_marker_cfp = RUBY_VM_END_CONTROL_FRAME(ec);
    size = end_marker_cfp - last_cfp;
    for (i = 0, fp = end_marker_cfp - 1; i < size; i++, fp = RUBY_VM_NEXT_CONTROL_FRAME(fp))
	if (fp->pc && (iseq = (rb_iseq_t *)fp->iseq) != NULL /* discards const */
	    && imemo_type((VALUE) iseq) == imemo_iseq
	    && (ui = iseq->body->unit_iseq) != NULL) {
	    ui->used_code_p = TRUE;
	}
}

/* Unload JIT code of some units to satisfy the maximum permitted
   number of units with a loaded code.  */
static void
unload_units(void)
{
    rb_vm_t *vm = GET_THREAD()->vm;
    rb_thread_t *th = 0;
    struct rb_mjit_unit *u, *next, *best_u;
    struct rb_mjit_unit_iseq *ui, *best_ui;
    unsigned long overall_calls, best_overall_calls;
    struct mjit_cont *cont;
    int n, units_num = active_units.length + obsolete_units.length;

    list_for_each(&vm->living_threads, th, vmlt_node) {
	mark_thread_unit_iseqs(th->ec);
    }
    for (cont = first_cont; cont != NULL; cont = cont->next) {
	mark_thread_unit_iseqs(cont->ec);
    }
    for (u = obsolete_units.head; u != NULL; u = next) {
	next = u->next;
	assert(u->unit_iseq != NULL && u->unit_iseq->unit != u);
	/* We can not just remove obsolete unit code.  Although it
	   will be never used, it might be still on the stack.  For
	   example, obsolete code might be still on the stack for
	   previous recursive calls.  */
	if (u->unit_iseq->used_code_p)
	    continue;
	verbose(2, "Unloading obsolete unit %d(%s)\n", u->num, u->unit_iseq->label);
	remove_from_list(u, &obsolete_units);
	/* ??? provide more parallelism */
	free_unit(u);
    }
    /* Remove 1/10 units more to decrease unloading calls.  */
    n = active_units.length / 10;
    for (; active_units.length + obsolete_units.length > mjit_opts.max_cache_size - n;) {
	best_u = NULL;
	best_ui = NULL;
	for (u = active_units.head; u != NULL; u = u->next) {
	    ui = u->unit_iseq;
	    assert(ui != NULL && ui->iseq != NULL && ui->unit == u && u->handle != NULL);
	    overall_calls = ui->iseq->body->resume_calls + ui->iseq->body->stop_calls;
	    if (ui->used_code_p
		|| (best_u != NULL && best_overall_calls < overall_calls))
		continue;
	    best_u = u;
	    best_ui = ui;
	    best_overall_calls = overall_calls;
	}
	if (best_u == NULL)
	    break;
	verbose(2, "Unloading unit %d(%s) (calls=%lu)",
		best_u->num, best_ui->label, best_overall_calls);
	assert(best_ui->iseq != NULL);
	best_ui->iseq->body->jit_code = (void *)(ptrdiff_t)NOT_READY_JIT_ISEQ_FUN;
	best_ui->iseq->body->stop_calls += best_ui->iseq->body->resume_calls;
	best_ui->iseq->body->resume_calls = 0;
	remove_from_list(best_u, &active_units);
	discard_unit(best_u);
    }
    verbose(1, "Too many JIT code -- %d units unloaded",
	    units_num - (active_units.length + obsolete_units.length));
}

/* Add ISEQ to be JITed in parallel with the current thread.  Add it
   to the current unit.  Add the current unit to the queue.  Unload
   some units if there are too many of them.  */
void
mjit_add_iseq_to_process(rb_iseq_t *iseq)
{
    struct rb_mjit_unit_iseq *ui;

    if (!mjit_init_p || stop_mjit_generation_p)
	return;
    create_iseq_unit(iseq);
    if ((ui = iseq->body->unit_iseq) == NULL)
	/* Failure in creating the unit iseq.  */
	return;
    verbose(2, "Adding iseq %s", ui->label);
    CRITICAL_SECTION_START(3, "in add_iseq_to_process");
    finish_forming_curr_unit();
    if (active_units.length + obsolete_units.length >= mjit_opts.max_cache_size) {
	/* Unload some units.  */
	mark_unit_iseqs(&obsolete_units);
	mark_unit_iseqs(&active_units);
	unload_units();
    }
    debug(3, "Sending wakeup signal to workers in mjit_add_iseq_to_process");
    native_cond_broadcast(&mjit_worker_wakeup);
    CRITICAL_SECTION_FINISH(3, "in add_iseq_to_process");
}


/* Iseqs can be garbage collected.  This function should call when it
   happens.  It removes unit iseq from the unit.  */
void
mjit_free_iseq(const rb_iseq_t *iseq)
{
    struct rb_mjit_unit_iseq *ui;
    struct rb_mjit_unit *u;

    if (!mjit_init_p || (ui = iseq->body->unit_iseq) == NULL)
	return;
    CRITICAL_SECTION_START(3, "to clear iseq in mjit_free_iseq");
    u = ui->unit;
    if (u == NULL) {
	/* Was obsoleted */
	CRITICAL_SECTION_FINISH(3, "to clear iseq in mjit_free_iseq");
	return;
    }
    ui->iseq = NULL;
    if (u->status == UNIT_IN_QUEUE) {
	remove_from_list(u, &unit_queue);
    } else if (u->status == UNIT_LOADED) {
	remove_from_list(u, &active_units);
    } else if (u->status == UNIT_IN_GENERATION) {
	u->freed_p = TRUE;
    }
    if (u->status != UNIT_IN_GENERATION)
	discard_unit(u);
    CRITICAL_SECTION_FINISH(3, "to clear iseq in mjit_free_iseq");
    if (mjit_opts.debug) {
	ui->resume_calls = iseq->body->resume_calls;
	ui->stop_calls = iseq->body->stop_calls;
    }
}

/* Wait until workers don't compile any iseq.  It is called at the
   start of GC.  */
void
mjit_gc_start(void)
{
    if (!mjit_init_p)
	return;
    debug(4, "mjit_gc_start");
    CRITICAL_SECTION_START(4, "mjit_gc_start");
    while (units_in_translation != 0) {
	debug(4, "Waiting wakeup from a worker for GC");
	native_cond_wait(&mjit_client_wakeup, &mjit_engine_mutex);
	debug(4, "Getting wakeup from a worker for GC");
    }
    in_gc = TRUE;
    CRITICAL_SECTION_FINISH(4, "mjit_gc_start");
}

/* Send a signal to workers to continue iseq compilations.  It is
   called at the end of GC.  */
void
mjit_gc_finish(void)
{
    if (!mjit_init_p)
	return;
    debug(4, "mjit_gc_finish");
    CRITICAL_SECTION_START(4, "mjit_gc_finish");
    in_gc = FALSE;
    debug(4, "Sending wakeup signal to workers after GC");
    native_cond_broadcast(&mjit_gc_wakeup);
    CRITICAL_SECTION_FINISH(4, "mjit_gc_finish");
}

/* Register a new continuation with thread TH.  Return MJIT info about
   the continuation.  */
struct mjit_cont *
mjit_cont_new(rb_execution_context_t *ec)
{
    return create_cont(ec);
}

/* Unregister continuation CONT.  */
void
mjit_cont_free(struct mjit_cont *cont)
{
    if (cont != NULL)
	free_cont(cont);
}

/* GCC and LLVM executable paths.  TODO: The paths should absolute
   ones to prevent changing C compiler for security reasons.  */
#define GCC_PATH "gcc"
#define LLVM_PATH "clang"

/* The default number of permitted ISEQ MJIT code mutations.  */
#define DEFAULT_MUTATIONS_NUM 2

/* This is called after each fork in the child in to switch off MJIT
   engine in the child as it does not inherit MJIT threads.  */
static void
child_after_fork(void)
{
    verbose(3, "Switching off MJIT in a forked child");
    mjit_init_p = FALSE;
    /* TODO: Should we initiate MJIT in the forked Ruby.  */
}

/* Initialize MJIT.  Start a thread creating the precompiled
   header.  Create worker threads processing units.  The function
   should be called first for using MJIT.  If everything is
   successfull, MJIT_INIT_P will be TRUE.  */
void
mjit_init(struct mjit_options *opts)
{
    FILE *f;
    const char *path;

    stop_mjit_generation_p = FALSE;
    in_gc = FALSE;
    units_in_translation = 0;
    mjit_opts = *opts;
    if (mjit_opts.max_cache_size <= 0)
	mjit_opts.max_cache_size = DEFAULT_CACHE_SIZE;
    if (mjit_opts.max_cache_size < MIN_CACHE_SIZE)
	mjit_opts.max_cache_size = MIN_CACHE_SIZE;
    mjit_time_start = real_ms_time();
    if (mjit_init_p)
	return;
    debug(2, "Start initializing MJIT");
    finish_worker_p = FALSE;
    worker_finished_p = FALSE;

    pch_fname = mjit_opts.debug ? MJIT_HEADER_BUILD_PATH "-debug.h.gch" : MJIT_HEADER_BUILD_PATH ".h.gch";
    if (f = fopen(pch_fname, "r")) {
	header_fname = mjit_opts.debug ? MJIT_HEADER_BUILD_PATH "-debug.h" : MJIT_HEADER_BUILD_PATH ".h";
    } else {
	pch_fname = mjit_opts.debug ? MJIT_HEADER_INSTALL_PATH "-debug.h.gch" : MJIT_HEADER_INSTALL_PATH ".h.gch";
	if (f = fopen(pch_fname, "r")) {
	    header_fname = mjit_opts.debug ? MJIT_HEADER_INSTALL_PATH "-debug.h" : MJIT_HEADER_INSTALL_PATH ".h";
	} else {
	    header_fname = NULL;
	    verbose(1, "Failed to find header file in '%s' or '%s'.", MJIT_HEADER_BUILD_PATH ".h.gch", MJIT_HEADER_INSTALL_PATH ".h.gch");
	    return;
	}
    }
    fclose(f);

    tmp_dirname = getenv("TMP"); /* For MinGW */
    if (tmp_dirname == NULL)
	tmp_dirname = "/tmp";

#ifdef __MACH__
    if (!mjit_opts.llvm) {
	if (mjit_opts.warnings || mjit_opts.verbose)
	    fprintf(stderr, "MJIT warning: we use only clang on Mac OS X\n");
	mjit_opts.llvm = 1;
    }
#endif
    path = mjit_opts.llvm ? LLVM_PATH : GCC_PATH;
    cc_path = xmalloc(strlen(path) + 1);
    if (cc_path == NULL) {
	return;
    }
    strcpy(cc_path, path);
    init_conts();
    init_workers();

    native_mutex_initialize(&mjit_engine_mutex);
    native_cond_initialize(&mjit_pch_wakeup, RB_CONDATTR_CLOCK_MONOTONIC);
    native_cond_initialize(&mjit_client_wakeup, RB_CONDATTR_CLOCK_MONOTONIC);
    native_cond_initialize(&mjit_worker_wakeup, RB_CONDATTR_CLOCK_MONOTONIC);
    native_cond_initialize(&mjit_gc_wakeup, RB_CONDATTR_CLOCK_MONOTONIC);

    if (rb_thread_create_mjit_thread(child_after_fork, worker)) {
	mjit_init_p = TRUE;
	verbose(1, "Successful MJIT initialization");
    } else {
 	native_mutex_destroy(&mjit_engine_mutex);
 	native_cond_destroy(&mjit_pch_wakeup);
 	native_cond_destroy(&mjit_client_wakeup);
 	native_cond_destroy(&mjit_worker_wakeup);
 	native_cond_destroy(&mjit_gc_wakeup);
 	verbose(1, "Failure in MJIT thread initialization\n");
    }
}

/* Finish the threads processing units and creating PCH, finalize
   and free MJIT data.  It should be called last during MJIT
   life.  */
void
mjit_finish(void)
{
    if (!mjit_init_p)
	return;
    debug(1, "Initiate finishing MJIT");
    /* As our threads are detached, we could just cancel them.  But it
       is a bad idea because OS processes (C compiler) started by
       threads can produce temp files.  And even if the temp files are
       removed, the used C compiler still complaint about their
       absence.  So wait for a clean finish of the threads.  */
    finish_worker_p = TRUE;
    while (!worker_finished_p) {
	debug(3, "Sending cancel signal to workers");
	CRITICAL_SECTION_START(3, "in mjit_finish");
	native_cond_broadcast(&mjit_worker_wakeup);
	CRITICAL_SECTION_FINISH(3, "in mjit_finish");
    }
    native_mutex_destroy(&mjit_engine_mutex);
    native_cond_destroy(&mjit_pch_wakeup);
    native_cond_destroy(&mjit_client_wakeup);
    native_cond_destroy(&mjit_worker_wakeup);
    native_cond_destroy(&mjit_gc_wakeup);
    free(cc_path); cc_path = NULL;
    finish_workers();
    finish_conts();
    mjit_init_p = FALSE;
    verbose(1, "Successful MJIT finish");
}
