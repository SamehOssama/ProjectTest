void RGB_set(uint8_t mask){
    mask &= 0X0E ;
    GPIO_PORTF_DATA_R=mask;
}
/**
 * @brief 
 * 
 * @param mask put 1 on the pin you want to clear
 */
void RGB_clear(uint8_t mask){
    mask &= 0X0E ;
    GPIO_PORTF_DATA_R &= ~mask;
}