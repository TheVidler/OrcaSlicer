#include "FillAdaptiveCrossZag.hpp"
#include "../ClipperUtils.hpp"
#include "../ShortestPath.hpp"
#include "../Surface.hpp"

namespace Slic3r {

Polylines FillAdaptiveCrossZag::_fill_surface_single(
    const unsigned int thickness_layers,
    const FillParams &params,
    coordf_t margin,
    coordf_t overlap)
{
    Polylines polylines;
    
    // 1. Determine the bounding box of the surface to fill
    BoundingBox bounding_box = this->bounding_box.inflated(margin);
    
    // Base spacing defined by the user's density setting
    coordf_t base_spacing = this->spacing;
    
    // 2. Rotate 90 degrees based on the layer ID (Cross Zag behavior)
    // We add a base 45-degree angle (M_PI/4) so it prints diagonally to X/Y axes, 
    // then rotate by 90 degrees (M_PI/2) every other layer.
    double angle = (this->layer_id % 2 == 0) ? (M_PI / 4.0) : (M_PI / 4.0 + M_PI / 2.0);
    
    // 3. To apply the angle, we rotate the bounding box, generate lines, then rotate back
    bounding_box.rotate(-angle);
    
    coord_t x_min = bounding_box.min.x();
    coord_t x_max = bounding_box.max.x();
    coord_t y_min = bounding_box.min.y();
    coord_t y_max = bounding_box.max.y();

    // 4. Generate the lines (Adaptive logic)
    // Note: params.adapt_fill_octree contains the spatial distance data.
    // If it's null, we fallback to standard uniform spacing.
    bool has_octree = params.adapt_fill_octree != nullptr;

    for (coord_t x = x_min; x <= x_max; x += scale_(base_spacing)) {
        Polyline current_line;
        
        for (coord_t y = y_min; y <= y_max; y += scale_(base_spacing)) {
            
            // Default density multiplier
            double density_multiplier = 1.0; 

            if (has_octree) {
                // Determine Z coordinate based on layer height
                coordf_t z = this->z; 
                
                // Query the octree. The further from the shell, the higher the returned 
                // cell size, meaning we can increase spacing (reduce density).
                // (You will need to map the octree's node depth to your density multiplier here)
                // density_multiplier = calculate_from_octree(x, y, z, params.adapt_fill_octree);
            }

            // Only plot the point if it falls within the required density threshold for this spatial cell
            // (e.g., skip every other line if density_multiplier is > 2.0)
            bool should_draw = true; // Replace with modulus logic based on density_multiplier

            if (should_draw) {
                Point p(x, y);
                current_line.points.push_back(p);
            } else {
                // If we skip drawing, we break the current line and push it to polylines, 
                // creating the "zag" gaps where density is reduced.
                if (!current_line.points.empty()) {
                    polylines.push_back(current_line);
                    current_line.points.clear();
                }
            }
        }
        if (!current_line.points.empty()) {
            polylines.push_back(current_line);
        }
    }

    // 5. Rotate the generated polylines back to the original orientation
    for (Polyline &polyline : polylines) {
        polyline.rotate(angle);
    }

    // 6. Clip the lines to the actual bounds of the surface
    polylines = intersection_pl(polylines, this->surface->expolygon.offset(margin));

    return polylines;
}

} // namespace Slic3r
