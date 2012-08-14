/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the 
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */


/**
 * @ingroup dataflow_components
 * @file
 * Buffer component
 * This file contains a buffer component which is the
 * most simple push-pull adapter.
 * The component accepts an event via a push input port
 * and sends the last received event for any request via
 * the pull output port.
 *
 * This may be useful for static spatial relationships which are
 * can be calibrated at runtime.
 *
 * @author Adnane Jadid <jadid@in.tum.de>
 */

#include <string>
#include <iostream>

#include <boost/bind.hpp>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utComponents/Sampler.h>

#include <utVision/Image.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Buffer component
 * This class contains a buffer component which is the
 * most simple push-pull adapter.
 * The component accepts an event via a push input port
 * and sends the last received event for any request via
 * the pull output port.
 *
 * This may be useful for static spatial relationships which are
 * can be calibrated at runtime.
 *
 * @par Input Ports
 * PushConsumer<EventType> port with name "Input".
 *
 * @par Output Ports
 * PullSupplier<EventType> port with name "Output".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * Whenever an event is received via the input port it is
 * buffered in an internal member variable.
 * Whenever an event is requested via the output port,
 * the last received event is replayed with an adapted
 * timestamp. If no event has been received so far
 * the output port cannot deliver.
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::Pose : PoseBuffer
 */


typedef Sampler< Measurement::ImageMeasurement > FrameSampler;


} } // namespace Ubitrack::Components

