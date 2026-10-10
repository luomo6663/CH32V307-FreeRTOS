#ifndef __WHEEL_ID_H
#define __WHEEL_ID_H

/*
 * Physical wheel numbering confirmed on the chassis:
 *   wheel 1 -> motor M0 -> right rear
 *   wheel 2 -> motor M1 -> right front
 *   wheel 3 -> motor M2 -> left front
 *   wheel 4 -> motor M3 -> left rear
 */
typedef enum
{
    WHEEL_ID_1_REAR_RIGHT = 0,
    WHEEL_ID_2_FRONT_RIGHT = 1,
    WHEEL_ID_3_FRONT_LEFT = 2,
    WHEEL_ID_4_REAR_LEFT = 3
} wheel_id_t;

#define WHEEL_COUNT     4U

#endif /* __WHEEL_ID_H */
