#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"

#define SHPR_INDEX(IRQn) (IRQn - 4)

void test_word_access(void)
{
    uint32_t *SHP_word = (uint32_t *)0xE000ED18;
    uint32_t value;

    SHP_word[0] = 100;
    SHP_word[1] = 200;
    SHP_word[2] = 300;

    value = SHP_word[0];
    value = SHP_word[1];
    value = SHP_word[2];
}

void test_byte_access(void)
{
    uint8_t value;

    /* Test writes to the System Handler Priority Registers registers */
    SCB->SHP[SHPR_INDEX(4)] = 4;
    SCB->SHP[SHPR_INDEX(5)] = 5;
    SCB->SHP[SHPR_INDEX(6)] = 6;
    SCB->SHP[SHPR_INDEX(11)] = 11;
    SCB->SHP[SHPR_INDEX(14)] = 14;
    SCB->SHP[SHPR_INDEX(15)] = 15;

    /* Test reads to the System Handler Priority Registers registers.  These
     * must be run in a debugger to view the results. */
    value = SCB->SHP[SHPR_INDEX(4)];
    value = SCB->SHP[SHPR_INDEX(5)];
    value = SCB->SHP[SHPR_INDEX(6)];
    value = SCB->SHP[SHPR_INDEX(11)];
    value = SCB->SHP[SHPR_INDEX(14)];
    value = SCB->SHP[SHPR_INDEX(15)];
}

int main(void)
{
    /* Freeze */
    while(1);
}
