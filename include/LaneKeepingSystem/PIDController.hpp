#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstdint>
#include <memory>

namespace Xycar {
/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class PIDController
{
public:
    using Ptr = std::unique_ptr<PIDController>;

    PIDController(PREC pGain, PREC iGain, PREC dGain);

    PREC getControlOutput(int32_t errorFromMid);

private:
    PREC kPGain;
    PREC kIGain;
    PREC kDGain;

    PREC pError;
    PREC iError;
    PREC dError;
};
} // namespace Xycar
#endif // PID_CONTROLLER_HPP_
