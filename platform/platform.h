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

uint64_t get_rx_timestamp_u64(void);
uint64_t get_tx_timestamp_u64(void);

uint64_t get_sys_timestamp_u64(void);
