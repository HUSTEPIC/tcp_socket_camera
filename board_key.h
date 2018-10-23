#ifndef BOARD_KEY_H
#define BOARD_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(HAL_KEY)

typedef void (*keysPressedCB_t)(uint8 keysPressed);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      Board_initKeys
 *
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 *
 * @return  none
 */
void Board_initKeys(keysPressedCB_t appKeyCB);

#endif

#ifdef __cplusplus
}
#endif

#endif /* BOARD_KEY_H */
