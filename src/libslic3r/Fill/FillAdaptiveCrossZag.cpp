#include "FillAdaptiveCrossZag.hpp"
#include "FillAdaptive.hpp" // Required to access the AdaptOctree structure
#include "../ClipperUtils.hpp"
#include "../ShortestPath.hpp"
#include "../Surface.hpp"
#include <cmath>

namespace Slic3r {

// Helper function to query the octree and return a density multiplier.
// Returns 1 for base density (near shells), 2 for half density, 4 for quarter, etc.
static int get_octree_multiplier(const AdaptOctree* octree, const Point& pt, coordf_t z)
{
    if (!octree) return 1;

    // Convert Slic3r scaled coordinates (coord_t) back to unscaled doubles for the octree
    Vec3d query_point(unscale<double>(pt.x()), unscale<double>(pt.y()), z);

    // Traverse the octree to find the leaf node containing this point.
    // Note: The exact node structure depends slightly on the OrcaSlicer version,
    // but the logic is to traverse down to the leaf.
    const AdaptOctree::Node* node = &octree->root;
    
    // Safety limit to prevent infinite loops, max depth is usually 5-8
    int max_depth = 10; 
    int current_depth = 0;

    while (!node->is_leaf() && current_depth < max_depth) {
        // Determine which of the 8 children contains the point
        int child_idx = 0;
        if (query_point.x() > node->center.x()) child_idx |= 1;
        if (query_point.y() > node->center.y()) child_idx |= 2;
        if (query_point.z() > node->center.z()) child_idx |= 4;
        
        if (node->children[child_idx] == nullptr) break;
        node = node->children[child_idx];
        current_depth++;
    }

    // In standard adaptive cubic, leaf depth defines density.
    // The deeper the node, the smaller the cube, meaning higher density (multiplier 1).
    // The shallower the node, the larger the cube, meaning lower density (multiplier > 1).
    // Assuming octree->max_depth is the densest level:
    int depth_diff = octree->max_depth - current_depth;
    
    // If depth_diff is 0, multiplier is 2^0 = 1 (draw every line)
    // If depth_diff is 1, multiplier is 2^1 = 2 (draw every 2nd line)
    // If depth_diff is 2, multiplier is 2^2 = 4 (draw every 4th line)
    return std::max(1, 1 << depth_diff); 
}

Polylines FillAdaptiveCrossZag::_fill_surface_single(
    const unsigned int thickness_layers,
    const FillParams &params,
    coordf_t margin,
    coordf_t overlap)
{
    Polylines polylines;
    
    // 1. Get the bounding box of the surface, inflated to ensure lines reach the perimeters
    BoundingBox bounding_box = this->bounding_box.inflated(margin);
    
    // Unscaled base spacing (usually in mm)
    coordf_t base_spacing = this->spacing;
    coord_t scaled_spacing = scale_(base_spacing);
    
    // 2. Determine Cross Zag angle based on layer ID.
    // We start at 45 degrees (M_PI/4) and alternate 90 degrees (M_PI/2) every layer.
    double angle = (this->layer_id % 2 == 0) ? (M_PI / 4.0) : (M_PI / 4.0 + M_PI / 2.0);
    
    // 3. Rotate the bounding box so we can generate simple vertical lines (parallel to Y)
    bounding_box.rotate(-angle);
    
    // We snap the X bounds to multiples of the spacing to ensure the grid aligns across layers
    coord_t x_min = bounding_box.min.x() - (bounding_box.min.x() % scaled_spacing) - scaled_spacing;
    coord_t x_max = bounding_box.max.x() + scaled_spacing - (bounding_box.max.x() % scaled_spacing);
    coord_t y_min = bounding_box.min.y() - scaled_spacing;
    coord_t y_max = bounding_box.max.y() + scaled_spacing;

    // Resolution for stepping along the Y axis to check the octree.
    // We check the 3D space every `base_spacing` distance.
    coord_t y_step = scaled_spacing; 

    // Pointer to the octree generated in PrintObject::prepare_infill()
    const AdaptOctree* octree = params.adapt_fill_octree;
    coordf_t z_height = this->z;

    // 4. Generate the lines
    for (coord_t x = x_min; x <= x_max; x += scaled_spacing) {
        
        // Calculate the absolute grid index for this X line.
        // This is critical: it ensures the math holds up across different coordinates.
        int line_index = std::abs((int)std::round((double)x / scaled_spacing));

        Polyline current_line;
        
        for (coord_t y = y_min; y <= y_max; y += y_step) {
            
            // To query the octree, we MUST transform this point back to the actual 
            // 3D world space (un-rotated).
            Point world_pt(x, y);
            world_pt.rotate(angle);

            int multiplier = get_octree_multiplier(octree, world_pt, z_height);

            // Modulus magic: If our line index is not a multiple of the allowed multiplier 
            // at this specific 3D location, we should NOT draw this segment.
            bool should_draw = (line_index % multiplier == 0);

            if (should_draw) {
                // Add the un-rotated point to the current segment
                current_line.points.push_back(Point(x, y));
            } else {
                // We've hit a low-density zone and shouldn't draw this line index here.
                // If we were drawing a line, break it off and save it.
                if (current_line.points.size() > 1) {
                    polylines.push_back(current_line);
                }
                current_line.points.clear();
            }
        }
        
        // Push any remaining line segment at the end of the Y loop
        if (current_line.points.size() > 1) {
            polylines.push_back(current_line);
        }
    }

    // 5. Rotate all generated polylines back to the original world orientation
    for (Polyline &polyline : polylines) {
        polyline.rotate(angle);
    }

    // 6. Clip the generated lines strictly to the requested surface bounds
    // intersection_pl handles clipping the overlapping segments efficiently.
    polylines = intersection_pl(polylines, this->surface->expolygon.offset(margin));

    // 7. Connect the ends of the lines to reduce travel moves (Standard Slic3r behavior)
    // This connects the "Zags" together at the edges
    connect_lines(polylines, this->surface->expolygon.offset(margin), this->spacing);

    return polylines;
}

} // namespace Slic3r
