/**
 * @file print_tenth.c
 *
 * @brief Prints every tenth index and the index after it.
 *
 * @author Paul Simbulan
 */
#include <stdio.h>

void Print_Tenth(void);

int main(void)
{
    Print_Tenth();
    return 0;
}

void Print_Tenth(void)
{
    for (int i = 10; i <= 51; i++)
    {
        int n = i % 10;
        if (n == 0)
            printf("Index %d: Every 10th iteration.\n", i);
        else if (n == 1)
            printf("Index %d: Following the 10th iteration.\n", i);
    }
}
