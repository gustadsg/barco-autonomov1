/*
 * Positioning_BLE.c
 *
 *  Created on: Nov 19, 2023
 *      Author: João
 */

#include "Positioning_BLE.h";

POSITIONING_BLE_Cartesian_Point_t POSITIONING_BLE_GetPosition(
		POSITIONING_BLE_Config_t *config) {
	const float d1 = config->depart_beacon.device.distance;
	const float d2 = config->arrival_beacon.device.distance;
	const float d3 = config->other_beacon.device.distance;

	const POSITIONING_BLE_Cartesian_Point_t p1 = config->depart_beacon.point;
	const POSITIONING_BLE_Cartesian_Point_t p2 = config->arrival_beacon.point;
	const POSITIONING_BLE_Cartesian_Point_t p3 = config->other_beacon.point;

	const float d1_sq = d1 * d1;
	const float d2_sq = d2 * d2;
	const float d3_sq = d3 * d3;

	const float x1_sq = p1.x * p1.x;
	const float x2_sq = p2.x * p2.x;
	const float x3_sq = p3.x * p3.x;

	const float y1_sq = p1.y * p1.y;
	const float y2_sq = p2.y * p2.y;
	const float y3_sq = p3.y * p3.y;

	const float A = -2 * p1.x + 2 * p2.x;
	const float B = -2 * p1.y + 2 * p2.y;
	const float C = d1_sq - d2_sq - x1_sq + x2_sq - y1_sq + y2_sq;
	const float D = -2 * p2.x + 2 * p3.x;
	const float E = -2 * p2.y + 2 * p3.y;
	const float F = d2_sq - d3_sq - x2_sq + x3_sq - y2_sq + y3_sq;

	POSITIONING_BLE_Cartesian_Point_t result;
	result.x = (C * E - F * B) / (E * A - B * D);
	result.y = (C * D - A * F) / (B * D - A * E);
	return result;
}
;
