/* Contract between ctr-recomp generated code and the host runtime (Azahar). */
#ifndef CTR_ABI_H
#define CTR_ABI_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CTR_ABI_VERSION 3

typedef struct CtrState CtrState;
/* One recompiled function. On entry s->r[15] (| thumb in cpsr.T) is the
 * function start or one of its resume points. It returns when the guest
 * returns from it, on an exit, or when control leaves code it covers; in all
 * cases s->r[15] and s->cpsr.T describe the next guest instruction. */
typedef void (*CtrFunction)(CtrState* s);

typedef struct CtrCallbacks {
    /* slow paths: page_table entry NULL, or an access crossing a page */
    uint8_t (*read8)(CtrState* s, uint32_t vaddr);
    uint16_t (*read16)(CtrState* s, uint32_t vaddr);
    uint32_t (*read32)(CtrState* s, uint32_t vaddr);
    uint64_t (*read64)(CtrState* s, uint32_t vaddr);
    void (*write8)(CtrState* s, uint32_t vaddr, uint8_t v);
    void (*write16)(CtrState* s, uint32_t vaddr, uint16_t v);
    void (*write32)(CtrState* s, uint32_t vaddr, uint32_t v);
    void (*write64)(CtrState* s, uint32_t vaddr, uint64_t v);
    /* exclusive monitor, shared with the other cores */
    uint8_t (*exclusive_read8)(CtrState* s, uint32_t vaddr);
    uint16_t (*exclusive_read16)(CtrState* s, uint32_t vaddr);
    uint32_t (*exclusive_read32)(CtrState* s, uint32_t vaddr);
    uint64_t (*exclusive_read64)(CtrState* s, uint32_t vaddr);
    /* return 0 on success, as STREX writes it */
    uint32_t (*exclusive_write8)(CtrState* s, uint32_t vaddr, uint8_t v);
    uint32_t (*exclusive_write16)(CtrState* s, uint32_t vaddr, uint16_t v);
    uint32_t (*exclusive_write32)(CtrState* s, uint32_t vaddr, uint32_t v);
    uint32_t (*exclusive_write64)(CtrState* s, uint32_t vaddr, uint64_t v);
    void (*clear_exclusive)(CtrState* s);
    /* r[] and cpsr are in s; may set s->exit */
    void (*svc)(CtrState* s, uint32_t imm);
    /* undefined/unpredictable instruction, breakpoint: s->r[15] = faulting pc; sets s->exit */
    void (*exception)(CtrState* s, uint32_t pc, uint32_t kind);
    /* coprocessor 15 beyond the c13 thread registers and barriers */
    uint32_t (*cp15_read)(CtrState* s, uint32_t opc1, uint32_t crn, uint32_t crm, uint32_t opc2);
    void (*cp15_write)(CtrState* s, uint32_t opc1, uint32_t crn, uint32_t crm, uint32_t opc2, uint32_t v);
    /* function holding the entry for guest address | 1 for Thumb, NULL when the module has none */
    CtrFunction (*lookup)(CtrState* s, uint32_t addr_thumb);
} CtrCallbacks;

struct CtrState {
    uint32_t r[16];        /* r[15]: guest pc of the next instruction on every exit */
    uint32_t cpsr;         /* complete CPSR: NZCVQ, GE, T, E, mode */
    uint32_t fpscr;        /* FPSCR incl. NZCV, length/stride, mode bits; the cumulative exception
                              bits (0x9F) also collect in the host FPSR while native code runs, and
                              the host folds them in (and clears FPSR) when it takes the state */
    uint32_t ext[64];      /* VFP registers as dynarmic ExtRegs: S0..S31 overlay D0..D15, D16..D31 */
    int64_t ticks;         /* remaining budget; code subtracts and leaves when <= 0 */
    uint32_t exit;         /* nonzero: unwind every native frame back to the dispatcher */
    uint32_t tpidrurw;     /* cp15 c13 c0 2 */
    uint32_t tpidruro;     /* cp15 c13 c0 3 */
    uint8_t* const* page_table;   /* 1 << 20 entries: host pointer to each 4 KiB guest page, NULL for slow path */
    const CtrCallbacks* cb;
    void* user;
};


typedef struct CtrEntry {
    uint32_t addr;         /* guest address | 1 for Thumb */
    uint32_t func;         /* index into ctr_functions */
} CtrEntry;

typedef struct CtrModule {
    uint32_t abi_version;
    uint64_t title_id;
    uint32_t text_base, text_size;
    uint8_t text_sha256[32];          /* of the code the module was built from */
    const CtrFunction* functions;
    uint32_t function_count;
    const CtrEntry* entries;          /* sorted by addr: function starts and resume points */
    uint32_t entry_count;
} CtrModule;

/* exported by each generated shared library */
const CtrModule* ctr_module(void);

#ifdef __cplusplus
}
#endif
#endif
