#include "marker.hpp"

namespace nxtar{

/******************************************************************************
 * MARKER CLASS METHODS.                                                      *
 ******************************************************************************/

    /**
     * Clear the points vector associated with this marker.
     */
    Marker::~Marker(){
        points.clear();
    }
}
