#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template <typename PREC>
PIDController<PREC>::PIDController(PREC pGain, PREC iGain, PREC dGain)
{
    kPGain = pGain;
    kIGain = iGain;
    kDGain = dGain;

    pError = static_cast<PREC>(0);
    iError = static_cast<PREC>(0);
    dError = static_cast<PREC>(0);
}

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(int32_t error)
{
    PREC cte = static_cast<PREC>(error);
    dError = cte - pError;
    pError = cte;
    iError += cte;
    return kPGain*pError + kIGain*iError + kDGain*dError;
}

template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar