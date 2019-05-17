#include "ctransforms.h"

void kul::ct_twist(
            const hom_t_t& newframe_H_current,
            const twist_t& velocity_current,
            twist_t& velocity_newframe)
{
    /* We need to perform a spatial motion vector coordinate transform, given
       the homogeneous coordinate transform for the same frames.
       Check e.g. chapter 2 of Roy's RBDA, for the relation between the two
       transforms, to understand this code
    */
    const auto& R = get_rotation_const( newframe_H_current ); // 3x3 rotation matrix
    const auto& w = angularCoords( velocity_current );        // omega in the current coordinates
    angularCoords( velocity_newframe ) = R * w;
    linearCoords ( velocity_newframe)  = R * linearCoords( velocity_current ) +
                           get_position_const( newframe_H_current).cross(
                                            angularCoords( velocity_newframe ) );
}
