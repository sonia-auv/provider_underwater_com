/**
 * \file	ModemM64_defitions.h
 * \author	Francis Alonzo <francisalonzo29@gmail.com
 * \date	06/09/2021
 * 
 * \copyright Copyright (c) 2021 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MODEMM64_DEFINITIONS
#define MODEMM64_DEFINITIONS

#define SOP "w"
#define EOP "\n"
#define DIR_CMD "c"
#define DIR_RESP "r"
#define CHECKSUM "*"

#define CMD_GET_VERSION "v"
#define CMD_GET_PAYLOAD_SIZE "n"
#define CMD_GET_BUFFER_LENGTH "l"
#define CMD_GET_DIAGNOSTIC "d"
#define CMD_GET_SETTINGS "c"
#define CMD_SET_SETTINGS "s"
#define CMD_QUEUE_PACKET "q"
#define CMD_FLUSH "f"
#define RESP_GOT_PACKET "p"

#endif // MODEMM64_DEFINITIONS