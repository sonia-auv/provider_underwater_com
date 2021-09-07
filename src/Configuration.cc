/**
 * \file	Configuration.cc
 * \author	Coumarc9
 * \date	24/07/2017
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

#include "Configuration.h"

namespace provider_underwater_com
{

    Configuration::Configuration(const ros::NodeHandlePtr &nh)
        : nh(nh),
          ttyPort("/dev/MODEM"),
          settingsFile("settings.txt")
    {
        Deserialize();
    }

    Configuration::~Configuration() {}

    void Configuration::Deserialize() {

        ROS_INFO("Deserialize params");

        FindParameter("/connection/tty_port", ttyPort);
        FindParameter("/settings/setting_file", settingsFile);

        ROS_INFO("End deserialize params");
    }

    template <typename TType>
    void Configuration::FindParameter(const std::string &paramName, TType &attribute) {
        if (nh->hasParam("/provider_underwater_com" + paramName)) {
            nh->getParam("/provider_underwater_com" + paramName, attribute);
        } else {
            ROS_WARN_STREAM("Did not find /provider_underwater_com" + paramName
                                    << ". Using default.");
        }
    }

}
