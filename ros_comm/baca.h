/**
 * @file baca.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Sending data through com port with BACA protocol
 * Info about BACA protocol at https://github.com/ctu-mrs/mrs_serial#mrs-serial-protocol
 * @version 0.1
 * @date 2022-11-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __BACA_H__
#define __BACA_H__

#define MESSAGE_ID 0xb0

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Write buffer to COM port
 * 
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 * @return uint8_t returns checksum of sent data
 */
uint8_t write_buf(void* data, int data_length);

/**
 * @brief Write buffer accoring to BACA protocol
 * 
 * @param data pointer to data
 * @param data_length length of the data in uint8_t size
 */
void write_baca(void* data, int data_length);

// TODO: receiver

#ifdef __cplusplus
}
#endif

#endif
