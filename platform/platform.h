/**
 * @brief Set slower SPI frequency
 * 
 */
void dwt_spi_set_rate_slow();
/**
 * @brief Set higher SPI frequency
 * 
 */
void dwt_spi_set_rate_fast();

void dwt_hardreset();

void dwt_hardinterrupt(void* callback);

void dwt_disable_interrupt();
void dwt_enable_interrupt();
