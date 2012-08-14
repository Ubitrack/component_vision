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

#include <utDataflow/ComponentFactory.h>
#include "FrameSampler.h"

namespace Ubitrack { namespace Components {

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< FrameSampler > ( "FrameSampler" );
}


} } // namespace Ubitrack::Components

