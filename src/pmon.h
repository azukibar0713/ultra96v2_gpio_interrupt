/*
 * pmon.h
 *
 *  Created on: 2020/07/26
 *      Author: Shun
 */

#ifndef SRC_PMON_H_
#define SRC_PMON_H_

/* Performance Monitor Control Register of Cortex A9*/
#define PMCR_D 3
#define PMCR_C 2
#define PMCR_E 0
#define PMCNTENSET_C 31

volatile inline static unsigned long __attribute__((always_inline))
pmon_start_cycle_counter()
{
    unsigned long x;

    x = 1 << PMCNTENSET_C;
    asm volatile("mcr	p15, 0, %0, c9, c12, 1" :: "r" (x));

    asm volatile("mrc	p15, 0, %0, c9, c12, 0" : "=r" (x));
    x |= ((1 << PMCR_D) | (1 << PMCR_C) | (1 << PMCR_E));
    x &= ~(1 << PMCR_D);
    asm volatile("mcr	p15, 0, %0, c9, c12, 0" :: "r" (x));

    asm volatile("mrc	p15, 0, %0, c9, c13, 0" : "=r" (x));
    return x;
}

volatile inline static unsigned long __attribute__((always_inline))
pmon_read_cycle_counter()
{
    unsigned long x;
    asm volatile ("mrc	p15, 0, %0, c9, c13, 0": "=r" (x));
    return x;
}


#if 0
// PMU���ғ���Ԃɐݒ�
volatile __inline__ static void __attribute__((always_inline))
pmu_start(void)
{
    //pmu_start
	asm volatile ("MRC    p15, 0, r0, c9, c12, 0"); // PMCR(�p�t�H�[�}���X���j�^���䃌�W�X�^)�����[�h
	asm volatile ("ORR    r0, r0, #0x1");           // E�r�b�g���Z�b�g(�ғ��ݒ�)
	asm volatile ("MCR    p15, 0, r0, c9, c12, 0"); // PMCR�����C�g
	asm volatile ("BX     lr");
}

// �T�C�N���J�E���^����l��ǂݍ���
volatile __inline__ static unsigned long __attribute__((always_inline))
pmu_read_counter(void)
{
    asm volatile ("MRC    p15, 0, r0, c9, c13, 0"); // ; PMCCNTR(�T�C�N���J�E���g���W�X�^)�����[�h
    asm volatile ("BX    lr");
}
#endif

#endif /* SRC_PMON_H_ */
