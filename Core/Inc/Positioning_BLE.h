/*
 * Positioning_BLE.h
 *
 *  Created on: Nov 19, 2023
 *      Author: João
 */

#ifndef INC_POSITIONING_BLE_H_
#define INC_POSITIONING_BLE_H_

#include "BLE-JDY18.h"

/**
 * @brief Represents a point in Cartesian coordinates for positioning using BLE.
 */
typedef struct {
    // @brief x-coordinate of the point.
    float x;
    // @brief y-coordinate of the point.
    float y;
} POSITIONING_BLE_Cartesian_Point_t;

/**
 * @brief Represents the location of a BLE beacon, including its device information.
 */
typedef struct {
    // @brief JDY18 BLE device information.
    JDY18_Device_t device;
    // @brief Cartesian coordinates of the beacon.
    POSITIONING_BLE_Cartesian_Point_t point;
} POSITIONING_BLE_Beacon_Location_t;

/**
 * @brief Represents the configuration for BLE-based positioning, including beacon locations.
 */
typedef struct {
    // @brief Location of the departure beacon.
    POSITIONING_BLE_Beacon_Location_t depart_beacon;
    // @brief Location of the arrival beacon.
    POSITIONING_BLE_Beacon_Location_t arrival_beacon;
    // @brief Location of another beacon.
    POSITIONING_BLE_Beacon_Location_t other_beacon;
} POSITIONING_BLE_Config_t;

/**
* @brief Gets the current position based on the provided BLE positioning configuration.
* @param config Pointer to the BLE positioning configuration.
* @return The Cartesian coordinates representing the current position.
*/
POSITIONING_BLE_Cartesian_Point_t POSITIONING_BLE_GetPosition(POSITIONING_BLE_Config_t *config);

#endif /* INC_POSITIONING_BLE_H_ */
