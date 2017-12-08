/**********************************************************************

  mjit.h - Interface to MRI method JIT compiler

  Copyright (C) 2017 Vladimir Makarov <vmakarov@redhat.com>.

**********************************************************************/

/* Special address values of a function generated from the
   corresponding iseq by MJIT: */
enum rb_mjit_iseq_fun {
    /* ISEQ was not queued yet for the machine code generation */
    NOT_ADDED_JIT_ISEQ_FUN = 0,
    /* ISEQ is already queued for the machine code generation but the
       code is not ready yet for the execution */
    NOT_READY_JIT_ISEQ_FUN = 1,
    /* End mark */
    LAST_JIT_ISEQ_FUN = 2,
};

/* A forward declaration  */
struct rb_mjit_unit_iseq;

/* MJIT options which can be defined on the MRI command line.  */
struct mjit_options {
    char on; /* flag of MJIT usage  */
    /* Flag to use LLVM Clang instead of default GCC for MJIT. TODO: remove */
    char llvm;
    /* Save temporary files after MRI finish.  The temporary files
       include the pre-compiled header, C code file generated for ISEQ,
       and the corresponding object file.  */
    char save_temps; /* TODO: imply this by debug */
    /* Print MJIT warnings to stderr.  */
    char warnings;
    /* Use debug mode.  It can be very slow as no optimizations are
       used.  */
    char debug;
    /* Force printing info about MJIT work of level VERBOSE or
       less.  */
    int verbose;
    /* Maximal permitted number of iseq JIT codes in a MJIT memory
       cache.  */
    int max_cache_size;
};

RUBY_SYMBOL_EXPORT_BEGIN
extern struct mjit_options mjit_opts;

/* Flag of successful MJIT initialization and intention to use it */
#ifdef MJIT_HEADER
static const int mjit_init_p = 1;
#else
extern int mjit_init_p;
#endif

extern void mjit_add_iseq_to_process(rb_iseq_t *iseq);
RUBY_SYMBOL_EXPORT_END

typedef VALUE (*mjit_fun_t)(rb_execution_context_t *, rb_control_frame_t *);

extern void mjit_init(struct mjit_options *opts);
extern void mjit_free_iseq(const rb_iseq_t *iseq);
extern void mjit_gc_start(void);
extern void mjit_gc_finish(void);
extern struct mjit_cont *mjit_cont_new(rb_execution_context_t *ec);
extern void mjit_cont_free(struct mjit_cont *cont);
extern void mjit_finish(void);

/* A threshold used to add iseq to JIT. */
#define NUM_CALLS_TO_ADD 5

/* A threshold used to reject long iseqs from JITting as such iseqs
   takes too much time to be compiled.  */
#define JIT_ISEQ_SIZE_THRESHOLD 1000

/* Try to execute the current ISEQ with BODY and TYPE in thread TH.
   Use JIT code if it is ready.  If it is not, add ISEQ to the
   compilation queue and return Qundef.  */
static inline VALUE
mjit_execute_iseq_0(rb_execution_context_t *ec, rb_iseq_t *iseq, struct rb_iseq_constant_body *body, int type)
{
    unsigned long n_calls;
    mjit_fun_t fun;
    VALUE v;

    fun = body->jit_code;
    n_calls = ++body->resume_calls;

    if (UNLIKELY((ptrdiff_t) fun <= (ptrdiff_t) LAST_JIT_ISEQ_FUN)) {
	switch ((enum rb_mjit_iseq_fun) fun) {
	case NOT_ADDED_JIT_ISEQ_FUN:
	    if (n_calls == NUM_CALLS_TO_ADD) {
		if ((type == ISEQ_TYPE_METHOD || type == ISEQ_TYPE_BLOCK)
		    && body->iseq_size < JIT_ISEQ_SIZE_THRESHOLD) {
		    body->jit_code = (void *) NOT_READY_JIT_ISEQ_FUN;
		    mjit_add_iseq_to_process(iseq);
		}
	    }
	    return Qundef;
	case NOT_READY_JIT_ISEQ_FUN:
	    return Qundef;
	default: /* To avoid a warning on LAST_JIT_ISEQ_FUN */
	    break;
	}
    }
    v = fun(ec, ec->cfp);
    return v;
}

/* See the above function.  */
static inline VALUE
mjit_execute_iseq(rb_execution_context_t *ec)
{
    rb_iseq_t *iseq;
    struct rb_iseq_constant_body *body;

    if (! mjit_init_p)
	return Qundef;
    iseq = (rb_iseq_t *)ec->cfp->iseq; /* discards const */
    body = iseq->body;
    return mjit_execute_iseq_0(ec, iseq, body, body->type);
}
