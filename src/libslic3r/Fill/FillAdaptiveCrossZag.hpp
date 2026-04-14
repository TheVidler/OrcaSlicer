#ifndef slic3r_FillAdaptiveCrossZag_hpp_
#define slic3r_FillAdaptiveCrossZag_hpp_

#include "FillBase.hpp"

namespace Slic3r {

class FillAdaptiveCrossZag : public Fill
{
public:
    Fill* clone() const override { return new FillAdaptiveCrossZag(*this); };
    ~FillAdaptiveCrossZag() override = default;
    
    // Whether this pattern can be sorted for optimal pathing
    bool use_bridge_flow() const override { return false; }
    bool no_sort() const override { return true; }

protected:
    // This is the core function where the geometry is generated
    Polylines _fill_surface_single(
        const unsigned int thickness_layers,
        const FillParams &params,
        coordf_t margin,
        coordf_t overlap) override;
};

} // namespace Slic3r

#endif // slic3r_FillAdaptiveCrossZag_hpp_
