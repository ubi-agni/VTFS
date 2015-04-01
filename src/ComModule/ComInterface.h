/*
    This file is part of VTFS--Visuo-Tactile-Force-Servoing.

    VTFS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    VTFS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CBF.  If not, see <http://www.gnu.org/licenses/>.


    Copyright 2009, 2010 Qiang Li
*/
#ifndef COMINTERFACE_H
#define COMINTERFACE_H

#include "UtilModule/RebaType.h"
//#define OKC_HOST "129.70.129.23"
#define OKC_HOST "192.168.10.123"
#define OKC_PORT "49938"
#define LEFTARM_IP "192.168.10.10"
#define RIGHTARM_IP "192.168.10.11"

class ComInterface
{
public:
    ComInterface();
    virtual void connect() = 0;
    virtual bool isConnected() = 0;
    virtual void waitForFinished() = 0;
    float cycle_time;
};

#endif // COMINTERFACE_H
