/*
 * This file is part of lslidar_c128 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LSLIDAR_C128_DECODER_H
#define LSLIDAR_C128_DECODER_H

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lslidar_c128_msgs/LslidarC128Packet.h>
#include <lslidar_c128_msgs/LslidarC128Point.h>
#include <lslidar_c128_msgs/LslidarC128Scan.h>
#include <lslidar_c128_msgs/LslidarC128Sweep.h>
#include <lslidar_c128_msgs/LslidarC128Layer.h>


namespace lslidar_c128_decoder {

// Raw lslidar packet constants and structures.
static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE =
        (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// valid packets with readings up to 200.0.
static const double DISTANCE_MAX        = 200.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.01; /**< meters */
static const double DISTANCE_MAX_UNITS  =
        (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK_ONE = 0xe0ff;   //flag 0
static const uint16_t UPPER_BANK_TWO =0xe1ff;    //flag 1
static const uint16_t UPPER_BANK_THREE=0xe2ff;    //flag 2
static const uint16_t UPPER_BANK_FOUR = 0xe3ff;  //flag 3

/** Special Defines for LSC128 support **/
static const int     FIRINGS_PER_BLOCK = 1;
static const int     SCANS_PER_FIRING  = 32;
static const double  BLOCK_TDURATION   = 110.592; // [µs]
static const double  DSR_TOFFSET       = 1;   
static const double  FIRING_TOFFSET    = 16;  

static const int PACKET_SIZE        = 1206;
static const int BLOCKS_PER_PACKET  = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET =
        (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static const int FIRINGS_PER_PACKET =
        FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
	static const double scan_one_altitude[32] = {
        -0.296705972839,-0.261799387799,
        -0.226892802759,-0.191986217719,
        -0.157079632679,-0.12217304764,
        -0.0872664625997,-0.0523598775598,
        -0.0174532925199,0.0174532925199, 
        0.0523598775598, 0.0872664625997, 
        0.12217304764,0.157079632679, 
        0.191986217719, 0.226892802759, 
		-0.266162710929,-0.231256125889,
        -0.196349540849,-0.161442955809,
        -0.12653637077,-0.0916297857297,
        -0.0567232006898,-0.0218166156499,
         0.01308996939,0.0479965544298,
         0.0829031394697,0.11780972451,
         0.15271630955,0.187622894589,
         0.222529479629,0.257436064669,
};
	
static const double cos_scan_one_altitude[32] = {
    std::cos(scan_one_altitude[ 0]), std::cos(scan_one_altitude[ 1]),
    std::cos(scan_one_altitude[ 2]), std::cos(scan_one_altitude[ 3]),
    std::cos(scan_one_altitude[ 4]), std::cos(scan_one_altitude[ 5]),
    std::cos(scan_one_altitude[ 6]), std::cos(scan_one_altitude[ 7]),
    std::cos(scan_one_altitude[ 8]), std::cos(scan_one_altitude[ 9]),
    std::cos(scan_one_altitude[10]), std::cos(scan_one_altitude[11]),
    std::cos(scan_one_altitude[12]), std::cos(scan_one_altitude[13]),
    std::cos(scan_one_altitude[14]), std::cos(scan_one_altitude[15]),
    std::cos(scan_one_altitude[16]), std::cos(scan_one_altitude[17]),
    std::cos(scan_one_altitude[18]), std::cos(scan_one_altitude[19]),
    std::cos(scan_one_altitude[20]), std::cos(scan_one_altitude[21]),
    std::cos(scan_one_altitude[22]), std::cos(scan_one_altitude[23]),
    std::cos(scan_one_altitude[24]), std::cos(scan_one_altitude[25]),
    std::cos(scan_one_altitude[26]), std::cos(scan_one_altitude[27]),
    std::cos(scan_one_altitude[28]), std::cos(scan_one_altitude[29]),
    std::cos(scan_one_altitude[30]), std::cos(scan_one_altitude[31]),
};

static const double sin_scan_one_altitude[32] = {
    std::sin(scan_one_altitude[ 0]), std::sin(scan_one_altitude[ 1]),
    std::sin(scan_one_altitude[ 2]), std::sin(scan_one_altitude[ 3]),
    std::sin(scan_one_altitude[ 4]), std::sin(scan_one_altitude[ 5]),
    std::sin(scan_one_altitude[ 6]), std::sin(scan_one_altitude[ 7]),
    std::sin(scan_one_altitude[ 8]), std::sin(scan_one_altitude[ 9]),
    std::sin(scan_one_altitude[10]), std::sin(scan_one_altitude[11]),
    std::sin(scan_one_altitude[12]), std::sin(scan_one_altitude[13]),
    std::sin(scan_one_altitude[14]), std::sin(scan_one_altitude[15]),
    std::sin(scan_one_altitude[16]), std::sin(scan_one_altitude[17]),
    std::sin(scan_one_altitude[18]), std::sin(scan_one_altitude[19]),
    std::sin(scan_one_altitude[20]), std::sin(scan_one_altitude[21]),
    std::sin(scan_one_altitude[22]), std::sin(scan_one_altitude[23]),
    std::sin(scan_one_altitude[24]), std::sin(scan_one_altitude[25]),
    std::sin(scan_one_altitude[26]), std::sin(scan_one_altitude[27]),
    std::sin(scan_one_altitude[28]), std::sin(scan_one_altitude[29]),
    std::sin(scan_one_altitude[30]), std::sin(scan_one_altitude[31]),
};

static const double scan_two_altitude[32] = {
        -0.292342649709,-0.257436064669,
        -0.222529479629,-0.187622894589,
        -0.15271630955,-0.11780972451,
        -0.0829031394697,-0.0479965544298,
        -0.01308996939, 0.0218166156499, 
         0.0567232006898, 0.0916297857297, 
         0.12653637077, 0.161442955809, 
         0.196349540849, 0.231256125889, 
	    -0.270526034059,-0.235619449019,
	    -0.200712863979,-0.165806278939,
	    -0.1308996939,-0.0959931088597,
	    -0.0610865238198,-0.0261799387799,
	    0.00872664625997,0.0436332312999,
	    0.0785398163397,0.11344640138,
	    0.14835298642,0.183259571459,
	    0.218166156499,0.253072741539,
};
static const double cos_scan_two_altitude[32] = {
    std::cos(scan_two_altitude[ 0]), std::cos(scan_two_altitude[ 1]),
    std::cos(scan_two_altitude[ 2]), std::cos(scan_two_altitude[ 3]),
    std::cos(scan_two_altitude[ 4]), std::cos(scan_two_altitude[ 5]),
    std::cos(scan_two_altitude[ 6]), std::cos(scan_two_altitude[ 7]),
    std::cos(scan_two_altitude[ 8]), std::cos(scan_two_altitude[ 9]),
    std::cos(scan_two_altitude[10]), std::cos(scan_two_altitude[11]),
    std::cos(scan_two_altitude[12]), std::cos(scan_two_altitude[13]),
    std::cos(scan_two_altitude[14]), std::cos(scan_two_altitude[15]),
    std::cos(scan_two_altitude[16]), std::cos(scan_two_altitude[17]),
    std::cos(scan_two_altitude[18]), std::cos(scan_two_altitude[19]),
    std::cos(scan_two_altitude[20]), std::cos(scan_two_altitude[21]),
    std::cos(scan_two_altitude[22]), std::cos(scan_two_altitude[23]),
    std::cos(scan_two_altitude[24]), std::cos(scan_two_altitude[25]),
    std::cos(scan_two_altitude[26]), std::cos(scan_two_altitude[27]),
    std::cos(scan_two_altitude[28]), std::cos(scan_two_altitude[29]),
    std::cos(scan_two_altitude[30]), std::cos(scan_two_altitude[31]),
};

static const double sin_scan_two_altitude[32] = {
    std::sin(scan_two_altitude[ 0]), std::sin(scan_two_altitude[ 1]),
    std::sin(scan_two_altitude[ 2]), std::sin(scan_two_altitude[ 3]),
    std::sin(scan_two_altitude[ 4]), std::sin(scan_two_altitude[ 5]),
    std::sin(scan_two_altitude[ 6]), std::sin(scan_two_altitude[ 7]),
    std::sin(scan_two_altitude[ 8]), std::sin(scan_two_altitude[ 9]),
    std::sin(scan_two_altitude[10]), std::sin(scan_two_altitude[11]),
    std::sin(scan_two_altitude[12]), std::sin(scan_two_altitude[13]),
    std::sin(scan_two_altitude[14]), std::sin(scan_two_altitude[15]),
    std::sin(scan_two_altitude[16]), std::sin(scan_two_altitude[17]),
    std::sin(scan_two_altitude[18]), std::sin(scan_two_altitude[19]),
    std::sin(scan_two_altitude[20]), std::sin(scan_two_altitude[21]),
    std::sin(scan_two_altitude[22]), std::sin(scan_two_altitude[23]),
    std::sin(scan_two_altitude[24]), std::sin(scan_two_altitude[25]),
    std::sin(scan_two_altitude[26]), std::sin(scan_two_altitude[27]),
    std::sin(scan_two_altitude[28]), std::sin(scan_two_altitude[29]),
    std::sin(scan_two_altitude[30]), std::sin(scan_two_altitude[31]),
};
static const double scan_three_altitude[32] = {
        -0.287979326579,-0.253072741539,
        -0.218166156499,-0.183259571459,
        -0.14835298642,-0.11344640138,
        -0.0785398163397,-0.0436332312999,
        -0.00872664625997, 0.0261799387799, 
        0.0610865238198, 0.0959931088597, 
        0.1308996939, 0.165806278939, 
        0.200712863979, 0.235619449019, 
	    -0.279252680319,-0.244346095279,
	    -0.209439510239,-0.174532925199,
	    -0.13962634016,-0.10471975512,
	    -0.0698131700798,-0.0349065850399,
	    0.0,0.0349065850399,
	    0.0698131700798,0.10471975512,
	    0.13962634016,0.174532925199,
	    0.209439510239,0.244346095279,
};
static const double cos_scan_three_altitude[32] = {
    std::cos(scan_three_altitude[ 0]), std::cos(scan_three_altitude[ 1]),
    std::cos(scan_three_altitude[ 2]), std::cos(scan_three_altitude[ 3]),
    std::cos(scan_three_altitude[ 4]), std::cos(scan_three_altitude[ 5]),
    std::cos(scan_three_altitude[ 6]), std::cos(scan_three_altitude[ 7]),
    std::cos(scan_three_altitude[ 8]), std::cos(scan_three_altitude[ 9]),
    std::cos(scan_three_altitude[10]), std::cos(scan_three_altitude[11]),
    std::cos(scan_three_altitude[12]), std::cos(scan_three_altitude[13]),
    std::cos(scan_three_altitude[14]), std::cos(scan_three_altitude[15]),
    std::cos(scan_three_altitude[16]), std::cos(scan_three_altitude[17]),
    std::cos(scan_three_altitude[18]), std::cos(scan_three_altitude[19]),
    std::cos(scan_three_altitude[20]), std::cos(scan_three_altitude[21]),
    std::cos(scan_three_altitude[22]), std::cos(scan_three_altitude[23]),
    std::cos(scan_three_altitude[24]), std::cos(scan_three_altitude[25]),
    std::cos(scan_three_altitude[26]), std::cos(scan_three_altitude[27]),
    std::cos(scan_three_altitude[28]), std::cos(scan_three_altitude[29]),
    std::cos(scan_three_altitude[30]), std::cos(scan_three_altitude[31]),
};

static const double sin_scan_three_altitude[32] = {
    std::sin(scan_three_altitude[ 0]), std::sin(scan_three_altitude[ 1]),
    std::sin(scan_three_altitude[ 2]), std::sin(scan_three_altitude[ 3]),
    std::sin(scan_three_altitude[ 4]), std::sin(scan_three_altitude[ 5]),
    std::sin(scan_three_altitude[ 6]), std::sin(scan_three_altitude[ 7]),
    std::sin(scan_three_altitude[ 8]), std::sin(scan_three_altitude[ 9]),
    std::sin(scan_three_altitude[10]), std::sin(scan_three_altitude[11]),
    std::sin(scan_three_altitude[12]), std::sin(scan_three_altitude[13]),
    std::sin(scan_three_altitude[14]), std::sin(scan_three_altitude[15]),
    std::sin(scan_three_altitude[16]), std::sin(scan_three_altitude[17]),
    std::sin(scan_three_altitude[18]), std::sin(scan_three_altitude[19]),
    std::sin(scan_three_altitude[20]), std::sin(scan_three_altitude[21]),
    std::sin(scan_three_altitude[22]), std::sin(scan_three_altitude[23]),
    std::sin(scan_three_altitude[24]), std::sin(scan_three_altitude[25]),
    std::sin(scan_three_altitude[26]), std::sin(scan_three_altitude[27]),
    std::sin(scan_three_altitude[28]), std::sin(scan_three_altitude[29]),
    std::sin(scan_three_altitude[30]), std::sin(scan_three_altitude[31]),
};
static const double scan_four_altitude[32] = {
        -0.283616003449, -0.248709418409,
        -0.213802833369, -0.178896248329,
        -0.14398966329, -0.10908307825,
        -0.0741764932098, -0.0392699081699,
	    -0.00436332312999,  0.0305432619099, 
        0.0654498469498, 0.10035643199, 
        0.13526301703,   0.170169602069, 
        0.205076187109,  0.239982772149, 
	    -0.274889357189,-0.239982772149,
	    -0.205076187109,-0.170169602069,
	    -0.13526301703,-0.10035643199,
	    -0.0654498469498,-0.0305432619099,
	    0.00436332312999,0.0392699081699,
	     0.0741764932098,0.10908307825,
	     0.14398966329,0.178896248329,
	     0.213802833369,0.248709418409,
};
static const double cos_scan_four_altitude[32] = {
    std::cos(scan_four_altitude[ 0]), std::cos(scan_four_altitude[ 1]),
    std::cos(scan_four_altitude[ 2]), std::cos(scan_four_altitude[ 3]),
    std::cos(scan_four_altitude[ 4]), std::cos(scan_four_altitude[ 5]),
    std::cos(scan_four_altitude[ 6]), std::cos(scan_four_altitude[ 7]),
    std::cos(scan_four_altitude[ 8]), std::cos(scan_four_altitude[ 9]),
    std::cos(scan_four_altitude[10]), std::cos(scan_four_altitude[11]),
    std::cos(scan_four_altitude[12]), std::cos(scan_four_altitude[13]),
    std::cos(scan_four_altitude[14]), std::cos(scan_four_altitude[15]),
    std::cos(scan_four_altitude[16]), std::cos(scan_four_altitude[17]),
    std::cos(scan_four_altitude[18]), std::cos(scan_four_altitude[19]),
    std::cos(scan_four_altitude[20]), std::cos(scan_four_altitude[21]),
    std::cos(scan_four_altitude[22]), std::cos(scan_four_altitude[23]),
    std::cos(scan_four_altitude[24]), std::cos(scan_four_altitude[25]),
    std::cos(scan_four_altitude[26]), std::cos(scan_four_altitude[27]),
    std::cos(scan_four_altitude[28]), std::cos(scan_four_altitude[29]),
    std::cos(scan_four_altitude[30]), std::cos(scan_four_altitude[31]),
};

static const double sin_scan_four_altitude[32] = {
    std::sin(scan_four_altitude[ 0]), std::sin(scan_four_altitude[ 1]),
    std::sin(scan_four_altitude[ 2]), std::sin(scan_four_altitude[ 3]),
    std::sin(scan_four_altitude[ 4]), std::sin(scan_four_altitude[ 5]),
    std::sin(scan_four_altitude[ 6]), std::sin(scan_four_altitude[ 7]),
    std::sin(scan_four_altitude[ 8]), std::sin(scan_four_altitude[ 9]),
    std::sin(scan_four_altitude[10]), std::sin(scan_four_altitude[11]),
    std::sin(scan_four_altitude[12]), std::sin(scan_four_altitude[13]),
    std::sin(scan_four_altitude[14]), std::sin(scan_four_altitude[15]),
    std::sin(scan_four_altitude[16]), std::sin(scan_four_altitude[17]),
    std::sin(scan_four_altitude[18]), std::sin(scan_four_altitude[19]),
    std::sin(scan_four_altitude[20]), std::sin(scan_four_altitude[21]),
    std::sin(scan_four_altitude[22]), std::sin(scan_four_altitude[23]),
    std::sin(scan_four_altitude[24]), std::sin(scan_four_altitude[25]),
    std::sin(scan_four_altitude[26]), std::sin(scan_four_altitude[27]),
    std::sin(scan_four_altitude[28]), std::sin(scan_four_altitude[29]),
    std::sin(scan_four_altitude[30]), std::sin(scan_four_altitude[31]),
};
	/*
static const double scan_one_altitude[32] = {
        -0.296705972839,-0.266162710929,
        -0.261799387799,-0.231256125889,
        -0.226892802759,-0.196349540849,
        -0.191986217719,-0.161442955809,
        -0.157079632679,-0.12653637077,
        -0.12217304764,-0.0916297857297,
        -0.0872664625997,-0.0567232006898,
        -0.0523598775598,-0.0218166156499,
        -0.0174532925199, 0.01308996939,
        0.0174532925199, 0.0479965544298,
        0.0523598775598, 0.0829031394697,
        0.0872664625997, 0.11780972451,
        0.12217304764, 0.15271630955,
        0.157079632679, 0.187622894589,
        0.191986217719, 0.222529479629,
	    0.226892802759, 0.257436064669,
};
static const double cos_scan_one_altitude[32] = {
    std::cos(scan_one_altitude[ 0]), std::cos(scan_one_altitude[ 1]),
    std::cos(scan_one_altitude[ 2]), std::cos(scan_one_altitude[ 3]),
    std::cos(scan_one_altitude[ 4]), std::cos(scan_one_altitude[ 5]),
    std::cos(scan_one_altitude[ 6]), std::cos(scan_one_altitude[ 7]),
    std::cos(scan_one_altitude[ 8]), std::cos(scan_one_altitude[ 9]),
    std::cos(scan_one_altitude[10]), std::cos(scan_one_altitude[11]),
    std::cos(scan_one_altitude[12]), std::cos(scan_one_altitude[13]),
    std::cos(scan_one_altitude[14]), std::cos(scan_one_altitude[15]),
    std::cos(scan_one_altitude[16]), std::cos(scan_one_altitude[17]),
    std::cos(scan_one_altitude[18]), std::cos(scan_one_altitude[19]),
    std::cos(scan_one_altitude[20]), std::cos(scan_one_altitude[21]),
    std::cos(scan_one_altitude[22]), std::cos(scan_one_altitude[23]),
    std::cos(scan_one_altitude[24]), std::cos(scan_one_altitude[25]),
    std::cos(scan_one_altitude[26]), std::cos(scan_one_altitude[27]),
    std::cos(scan_one_altitude[28]), std::cos(scan_one_altitude[29]),
    std::cos(scan_one_altitude[30]), std::cos(scan_one_altitude[31]),
};

static const double sin_scan_one_altitude[32] = {
    std::sin(scan_one_altitude[ 0]), std::sin(scan_one_altitude[ 1]),
    std::sin(scan_one_altitude[ 2]), std::sin(scan_one_altitude[ 3]),
    std::sin(scan_one_altitude[ 4]), std::sin(scan_one_altitude[ 5]),
    std::sin(scan_one_altitude[ 6]), std::sin(scan_one_altitude[ 7]),
    std::sin(scan_one_altitude[ 8]), std::sin(scan_one_altitude[ 9]),
    std::sin(scan_one_altitude[10]), std::sin(scan_one_altitude[11]),
    std::sin(scan_one_altitude[12]), std::sin(scan_one_altitude[13]),
    std::sin(scan_one_altitude[14]), std::sin(scan_one_altitude[15]),
    std::sin(scan_one_altitude[16]), std::sin(scan_one_altitude[17]),
    std::sin(scan_one_altitude[18]), std::sin(scan_one_altitude[19]),
    std::sin(scan_one_altitude[20]), std::sin(scan_one_altitude[21]),
    std::sin(scan_one_altitude[22]), std::sin(scan_one_altitude[23]),
    std::sin(scan_one_altitude[24]), std::sin(scan_one_altitude[25]),
    std::sin(scan_one_altitude[26]), std::sin(scan_one_altitude[27]),
    std::sin(scan_one_altitude[28]), std::sin(scan_one_altitude[29]),
    std::sin(scan_one_altitude[30]), std::sin(scan_one_altitude[31]),
};

static const double scan_two_altitude[32] = {
        -0.292342649709,-0.270526034059,
        -0.257436064669,-0.235619449019,
        -0.222529479629,-0.200712863979,
        -0.187622894589,-0.165806278939,
        -0.15271630955,-0.1308996939,
        -0.11780972451,-0.0959931088597,
        -0.0829031394697,-0.0610865238198,
        -0.0479965544298,-0.0261799387799,
        -0.01308996939, 0.00872664625997,
        0.0218166156499, 0.0436332312999,
        0.0567232006898, 0.0785398163397,
        0.0916297857297, 0.11344640138,
        0.12653637077, 0.14835298642,
        0.161442955809, 0.183259571459,
        0.196349540849, 0.218166156499,
	    0.231256125889, 0.253072741539,
};
static const double cos_scan_two_altitude[32] = {
    std::cos(scan_two_altitude[ 0]), std::cos(scan_two_altitude[ 1]),
    std::cos(scan_two_altitude[ 2]), std::cos(scan_two_altitude[ 3]),
    std::cos(scan_two_altitude[ 4]), std::cos(scan_two_altitude[ 5]),
    std::cos(scan_two_altitude[ 6]), std::cos(scan_two_altitude[ 7]),
    std::cos(scan_two_altitude[ 8]), std::cos(scan_two_altitude[ 9]),
    std::cos(scan_two_altitude[10]), std::cos(scan_two_altitude[11]),
    std::cos(scan_two_altitude[12]), std::cos(scan_two_altitude[13]),
    std::cos(scan_two_altitude[14]), std::cos(scan_two_altitude[15]),
    std::cos(scan_two_altitude[16]), std::cos(scan_two_altitude[17]),
    std::cos(scan_two_altitude[18]), std::cos(scan_two_altitude[19]),
    std::cos(scan_two_altitude[20]), std::cos(scan_two_altitude[21]),
    std::cos(scan_two_altitude[22]), std::cos(scan_two_altitude[23]),
    std::cos(scan_two_altitude[24]), std::cos(scan_two_altitude[25]),
    std::cos(scan_two_altitude[26]), std::cos(scan_two_altitude[27]),
    std::cos(scan_two_altitude[28]), std::cos(scan_two_altitude[29]),
    std::cos(scan_two_altitude[30]), std::cos(scan_two_altitude[31]),
};

static const double sin_scan_two_altitude[32] = {
    std::sin(scan_two_altitude[ 0]), std::sin(scan_two_altitude[ 1]),
    std::sin(scan_two_altitude[ 2]), std::sin(scan_two_altitude[ 3]),
    std::sin(scan_two_altitude[ 4]), std::sin(scan_two_altitude[ 5]),
    std::sin(scan_two_altitude[ 6]), std::sin(scan_two_altitude[ 7]),
    std::sin(scan_two_altitude[ 8]), std::sin(scan_two_altitude[ 9]),
    std::sin(scan_two_altitude[10]), std::sin(scan_two_altitude[11]),
    std::sin(scan_two_altitude[12]), std::sin(scan_two_altitude[13]),
    std::sin(scan_two_altitude[14]), std::sin(scan_two_altitude[15]),
    std::sin(scan_two_altitude[16]), std::sin(scan_two_altitude[17]),
    std::sin(scan_two_altitude[18]), std::sin(scan_two_altitude[19]),
    std::sin(scan_two_altitude[20]), std::sin(scan_two_altitude[21]),
    std::sin(scan_two_altitude[22]), std::sin(scan_two_altitude[23]),
    std::sin(scan_two_altitude[24]), std::sin(scan_two_altitude[25]),
    std::sin(scan_two_altitude[26]), std::sin(scan_two_altitude[27]),
    std::sin(scan_two_altitude[28]), std::sin(scan_two_altitude[29]),
    std::sin(scan_two_altitude[30]), std::sin(scan_two_altitude[31]),
};
static const double scan_three_altitude[32] = {
        -0.287979326579,-0.279252680319,
        -0.253072741539,-0.244346095279,
        -0.218166156499,-0.209439510239,
        -0.183259571459,-0.174532925199,
        -0.14835298642,-0.13962634016,
        -0.11344640138,-0.10471975512,
        -0.0785398163397,-0.0698131700798,
        -0.0436332312999,-0.0349065850399,
        -0.00872664625997, 0.0,
        0.0261799387799, 0.0349065850399,
        0.0610865238198, 0.0698131700798,
        0.0959931088597, 0.10471975512,
        0.1308996939, 0.13962634016,
	    0.165806278939, 0.174532925199,
        0.200712863979, 0.209439510239,
        0.235619449019, 0.244346095279,
};
static const double cos_scan_three_altitude[32] = {
    std::cos(scan_three_altitude[ 0]), std::cos(scan_three_altitude[ 1]),
    std::cos(scan_three_altitude[ 2]), std::cos(scan_three_altitude[ 3]),
    std::cos(scan_three_altitude[ 4]), std::cos(scan_three_altitude[ 5]),
    std::cos(scan_three_altitude[ 6]), std::cos(scan_three_altitude[ 7]),
    std::cos(scan_three_altitude[ 8]), std::cos(scan_three_altitude[ 9]),
    std::cos(scan_three_altitude[10]), std::cos(scan_three_altitude[11]),
    std::cos(scan_three_altitude[12]), std::cos(scan_three_altitude[13]),
    std::cos(scan_three_altitude[14]), std::cos(scan_three_altitude[15]),
    std::cos(scan_three_altitude[16]), std::cos(scan_three_altitude[17]),
    std::cos(scan_three_altitude[18]), std::cos(scan_three_altitude[19]),
    std::cos(scan_three_altitude[20]), std::cos(scan_three_altitude[21]),
    std::cos(scan_three_altitude[22]), std::cos(scan_three_altitude[23]),
    std::cos(scan_three_altitude[24]), std::cos(scan_three_altitude[25]),
    std::cos(scan_three_altitude[26]), std::cos(scan_three_altitude[27]),
    std::cos(scan_three_altitude[28]), std::cos(scan_three_altitude[29]),
    std::cos(scan_three_altitude[30]), std::cos(scan_three_altitude[31]),
};

static const double sin_scan_three_altitude[32] = {
    std::sin(scan_three_altitude[ 0]), std::sin(scan_three_altitude[ 1]),
    std::sin(scan_three_altitude[ 2]), std::sin(scan_three_altitude[ 3]),
    std::sin(scan_three_altitude[ 4]), std::sin(scan_three_altitude[ 5]),
    std::sin(scan_three_altitude[ 6]), std::sin(scan_three_altitude[ 7]),
    std::sin(scan_three_altitude[ 8]), std::sin(scan_three_altitude[ 9]),
    std::sin(scan_three_altitude[10]), std::sin(scan_three_altitude[11]),
    std::sin(scan_three_altitude[12]), std::sin(scan_three_altitude[13]),
    std::sin(scan_three_altitude[14]), std::sin(scan_three_altitude[15]),
    std::sin(scan_three_altitude[16]), std::sin(scan_three_altitude[17]),
    std::sin(scan_three_altitude[18]), std::sin(scan_three_altitude[19]),
    std::sin(scan_three_altitude[20]), std::sin(scan_three_altitude[21]),
    std::sin(scan_three_altitude[22]), std::sin(scan_three_altitude[23]),
    std::sin(scan_three_altitude[24]), std::sin(scan_three_altitude[25]),
    std::sin(scan_three_altitude[26]), std::sin(scan_three_altitude[27]),
    std::sin(scan_three_altitude[28]), std::sin(scan_three_altitude[29]),
    std::sin(scan_three_altitude[30]), std::sin(scan_three_altitude[31]),
};
static const double scan_four_altitude[32] = {
        -0.283616003449,-0.274889357189,
        -0.248709418409,-0.239982772149,
        -0.213802833369,-0.205076187109,
        -0.178896248329,-0.170169602069,
        -0.14398966329,-0.13526301703,
        -0.10908307825,-0.10035643199,
        -0.0741764932098,-0.0654498469498,
        -0.0392699081699,-0.0305432619099,
	    -0.00436332312999, 0.00436332312999,
        0.0305432619099, 0.0392699081699,
        0.0654498469498, 0.0741764932098,
        0.10035643199, 0.10908307825,
        0.13526301703, 0.14398966329,
        0.170169602069, 0.178896248329,
        0.205076187109, 0.213802833369,
        0.239982772149, 0.248709418409,
};
static const double cos_scan_four_altitude[32] = {
    std::cos(scan_four_altitude[ 0]), std::cos(scan_four_altitude[ 1]),
    std::cos(scan_four_altitude[ 2]), std::cos(scan_four_altitude[ 3]),
    std::cos(scan_four_altitude[ 4]), std::cos(scan_four_altitude[ 5]),
    std::cos(scan_four_altitude[ 6]), std::cos(scan_four_altitude[ 7]),
    std::cos(scan_four_altitude[ 8]), std::cos(scan_four_altitude[ 9]),
    std::cos(scan_four_altitude[10]), std::cos(scan_four_altitude[11]),
    std::cos(scan_four_altitude[12]), std::cos(scan_four_altitude[13]),
    std::cos(scan_four_altitude[14]), std::cos(scan_four_altitude[15]),
    std::cos(scan_four_altitude[16]), std::cos(scan_four_altitude[17]),
    std::cos(scan_four_altitude[18]), std::cos(scan_four_altitude[19]),
    std::cos(scan_four_altitude[20]), std::cos(scan_four_altitude[21]),
    std::cos(scan_four_altitude[22]), std::cos(scan_four_altitude[23]),
    std::cos(scan_four_altitude[24]), std::cos(scan_four_altitude[25]),
    std::cos(scan_four_altitude[26]), std::cos(scan_four_altitude[27]),
    std::cos(scan_four_altitude[28]), std::cos(scan_four_altitude[29]),
    std::cos(scan_four_altitude[30]), std::cos(scan_four_altitude[31]),
};

static const double sin_scan_four_altitude[32] = {
    std::sin(scan_four_altitude[ 0]), std::sin(scan_four_altitude[ 1]),
    std::sin(scan_four_altitude[ 2]), std::sin(scan_four_altitude[ 3]),
    std::sin(scan_four_altitude[ 4]), std::sin(scan_four_altitude[ 5]),
    std::sin(scan_four_altitude[ 6]), std::sin(scan_four_altitude[ 7]),
    std::sin(scan_four_altitude[ 8]), std::sin(scan_four_altitude[ 9]),
    std::sin(scan_four_altitude[10]), std::sin(scan_four_altitude[11]),
    std::sin(scan_four_altitude[12]), std::sin(scan_four_altitude[13]),
    std::sin(scan_four_altitude[14]), std::sin(scan_four_altitude[15]),
    std::sin(scan_four_altitude[16]), std::sin(scan_four_altitude[17]),
    std::sin(scan_four_altitude[18]), std::sin(scan_four_altitude[19]),
    std::sin(scan_four_altitude[20]), std::sin(scan_four_altitude[21]),
    std::sin(scan_four_altitude[22]), std::sin(scan_four_altitude[23]),
    std::sin(scan_four_altitude[24]), std::sin(scan_four_altitude[25]),
    std::sin(scan_four_altitude[26]), std::sin(scan_four_altitude[27]),
    std::sin(scan_four_altitude[28]), std::sin(scan_four_altitude[29]),
    std::sin(scan_four_altitude[30]), std::sin(scan_four_altitude[31]),
};
*/

	/*
static const double scan_one_altitude[16] = {
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494
};

static const double cos_scan_altitude[16] = {
    std::cos(scan_altitude[ 0]), std::cos(scan_altitude[ 1]),
    std::cos(scan_altitude[ 2]), std::cos(scan_altitude[ 3]),
    std::cos(scan_altitude[ 4]), std::cos(scan_altitude[ 5]),
    std::cos(scan_altitude[ 6]), std::cos(scan_altitude[ 7]),
    std::cos(scan_altitude[ 8]), std::cos(scan_altitude[ 9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
};

static const double sin_scan_altitude[16] = {
    std::sin(scan_altitude[ 0]), std::sin(scan_altitude[ 1]),
    std::sin(scan_altitude[ 2]), std::sin(scan_altitude[ 3]),
    std::sin(scan_altitude[ 4]), std::sin(scan_altitude[ 5]),
    std::sin(scan_altitude[ 6]), std::sin(scan_altitude[ 7]),
    std::sin(scan_altitude[ 8]), std::sin(scan_altitude[ 9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
};
*/
typedef struct{
    double distance;
    double intensity;
}point_struct;

struct PointXYZIT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

class LslidarC128Decoder {
public:

    LslidarC128Decoder(ros::NodeHandle& n, ros::NodeHandle& pn);
    LslidarC128Decoder(const LslidarC128Decoder&) = delete;
    LslidarC128Decoder operator=(const LslidarC128Decoder&) = delete;
    ~LslidarC128Decoder() {return;}

    bool initialize();

    typedef boost::shared_ptr<LslidarC128Decoder> LslidarC128DecoderPtr;
    typedef boost::shared_ptr<const LslidarC128Decoder> LslidarC128DecoderConstPtr;

private:

    union TwoBytes {
        uint16_t distance;
        uint8_t  bytes[2];
    };

    struct RawBlock {
        uint16_t header;        ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
        uint8_t  data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
    };

    struct Firing {
        // Azimuth associated with the first shot within this firing.
		double encoder_azimuth;   
        double firing_azimuth;
        double azimuth[SCANS_PER_FIRING];
        double distance[SCANS_PER_FIRING];
        double intensity[SCANS_PER_FIRING];
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();


    // Callback function for a single lslidar packet.
    int checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void layerCallback(const std_msgs::Int8Ptr& msg);
    void packetCallback(const lslidar_c128_msgs::LslidarC128PacketConstPtr& msg);
    // Publish data
    void publishPointCloud();
    void publishChannelScan();
    // Publish scan Data
    void publishScan();

    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        // According to the user manual,
        // azimuth = raw_azimuth / 100.0;
        //return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
		return static_cast<double>(raw_azimuth) / 100.0;
    }

    // calc the means_point
    point_struct getMeans(std::vector<point_struct> clusters);

    // configuration degree base
    int point_num;
    double angle_base;

    // Configuration parameters
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double angle3_disable_min;
    double angle3_disable_max;
    double frequency;
    bool publish_point_cloud;
    bool use_gps_ts;
    bool publish_scan;
    bool apollo_interface;
    double cos_azimuth_table[6300];
    double sin_azimuth_table[6300];

    bool is_first_sweep;
    double last_azimuth;
    double sweep_start_time;
    double packet_start_time;
    int layer_num;
    Firing firings[FIRINGS_PER_PACKET];
   // Firing firingsOne[FIRINGS_PER_PACKET];
   // Firing firingsTwo[FIRINGS_PER_PACKET];
   //	Firing firingsThree[FIRINGS_PER_PACKET];
   //	Firing firingsFour[FIRINGS_PER_PACKET];
    // ROS related parameters
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    //std::string fixed_frame_id;
    std::string frame_id;

    lslidar_c128_msgs::LslidarC128SweepPtr sweep_data;
    lslidar_c128_msgs::LslidarC128LayerPtr multi_scan;
    sensor_msgs::PointCloud2 point_cloud_data;

    ros::Subscriber packet_sub;
    ros::Subscriber layer_sub;
    ros::Publisher sweep_pub;
    ros::Publisher point_cloud_pub;
    ros::Publisher scan_pub;
    ros::Publisher channel_scan_pub;

};

typedef LslidarC128Decoder::LslidarC128DecoderPtr LslidarC128DecoderPtr;
typedef LslidarC128Decoder::LslidarC128DecoderConstPtr LslidarC128DecoderConstPtr;
typedef PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

} // end namespace lslidar_c128_decoder


POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_c128_decoder::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))
#endif
