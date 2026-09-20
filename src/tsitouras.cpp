#include <algorithm>
#include <cmath>

#include "bclibc/tsitouras.hpp"
#include "bclibc/embedded_rk45.hpp"

namespace bclibc
{
    namespace
    {
        // Tsitouras 5(4) ("Tsit5"): Ch. Tsitouras, "Runge-Kutta pairs of order
        // 5(4) satisfying only the first column simplifying assumption",
        // Computers & Mathematics with Applications 62(2), 2011, pp. 770-775.
        // Same 7-stage FSAL shape as Dormand-Prince (see dormand_prince.cpp):
        // stages 0-5 are the 6-stage core, stage 6 reuses the FSAL trick (its
        // A-row equals the 5th-order B weights) both to seed the next step's
        // first stage and to supply the 7th term the embedded error estimate
        // needs. Coefficients match ARKODE_TSITOURAS_7_4_5 in SUNDIALS/ARKODE
        // (include/arkode/arkode_butcher_erk.def) and the widely used "Tsit5"
        // tableau (e.g. OrdinaryDiffEq.jl's Tsit5Tableau).
        //
        // Free-standing (not class-static) constexpr arrays -- see
        // cash_karp.cpp's identical comment for why: a `static constexpr`
        // array class member here trips a GCC PIC-relocation quirk when
        // linked into a shared object.
        //
        // Row i-1 holds the A-coefficients for stage i (i=1..6).
        constexpr double kTsA[6][6] = {
            {0.161, 0.0, 0.0, 0.0, 0.0, 0.0},
            {-0.008480655492356988544426874250230774675121, 0.3354806554923569885444268742502307746751, 0.0, 0.0, 0.0, 0.0},
            {2.897153057105493432130432594192938764925, -6.359448489975074843148159912383825625953, 4.362295432869581411017727318190886861028, 0.0, 0.0, 0.0},
            {5.325864828439256604428877920840511317836, -11.74888356406282787774717033978577296189, 7.495539342889836208304604784564358155659, -0.0924950663617552492565020793320719161135, 0.0, 0.0},
            {5.861455442946420028659251486982647890394, -12.92096931784710929170611868178335939542, 8.159367898576158643180400794539253485182, -0.07158497328140099722453054252582973869127, -0.02826905039406838290900305721271224146718, 0.0},
            {0.09646076681806522951816731316512876333712, 0.01, 0.479889650414499574775249532290596519913, 1.379008574103741893192274821856872770756, -3.290069515436080679901047585711363850116, 2.324710524099773982415355918398765796109},
        };

        // 5th-order solution weights; stage 6 (the FSAL point) contributes
        // nothing further since the state is already fully determined by
        // stages 0-5 through row 6 of kTsA above.
        constexpr double kTsB[7] = {
            0.09646076681806522951816731316512876333712, 0.01, 0.479889650414499574775249532290596519913,
            1.379008574103741893192274821856872770756, -3.290069515436080679901047585711363850116,
            2.324710524099773982415355918398765796109, 0.0};

        // Embedded 4th-order weights.
        constexpr double kTsD[7] = {
            0.09468075576583945807478876255758922856118, 0.009183565540343253096776363936645313759814,
            0.4877705284247615707855642599631228241517, 1.234297566930478985655109673884237654036,
            -2.707712349983525454881109975059321670690, 1.866628418170587035753719399566211498666,
            0.01515151515151515151515151515151515151515};

        // Error weights: 5th-order minus embedded 4th-order.
        constexpr double kTsE[7] = {
            kTsB[0] - kTsD[0], kTsB[1] - kTsD[1], kTsB[2] - kTsD[2], kTsB[3] - kTsD[3],
            kTsB[4] - kTsD[4], kTsB[5] - kTsD[5], kTsB[6] - kTsD[6]};

        struct Tsitouras54Tableau
        {
            static constexpr int stages = 7;
            static constexpr bool fsal = true;

            static inline double A(int i, int j) { return kTsA[i - 1][j]; }
            static inline double B(int i) { return kTsB[i]; }
            static inline double E(int i) { return kTsE[i]; }
        };

        // Same controller as Dormand-Prince (see dormand_prince.cpp): matches
        // scipy.integrate._ivp.rk.RungeKutta's step-size controller (exponent
        // -1/(error_estimator_order+1) = -1/5 for a 4th-order embedded
        // estimate, safety 0.9, factor clamped to [0.2, 10], growth capped at
        // 1x for one step immediately following a rejection), and the same
        // wind-boundary step limiting. Appropriate for any 7-stage FSAL 5(4)
        // pair, not specific to Dormand-Prince's own coefficients.
        struct TsitourasController
        {
            static constexpr double kSafety = 0.9;
            static constexpr double kMinFactor = 0.2;
            static constexpr double kMaxFactor = 10.0;
            static constexpr double kErrorExponent = -1.0 / 5.0;

            static inline void on_wind_change(const BCLIBC_V3dT &velocity, const BCLIBC_V3dT &wind, BCLIBC_V3dT &vr)
            {
                vr = velocity - wind;
            }

            static inline void limit_step_at_wind_boundary(
                double next_range, const BCLIBC_V3dT &pos, const BCLIBC_V3dT &vr,
                const BCLIBC_V3dT &wind, double &dt)
            {
                const double ground_vx = vr.x + wind.x;
                if (ground_vx <= 0.0)
                    return;
                const double remaining = next_range - pos.x;
                if (remaining <= 0.0)
                    return;
                const double time_to_boundary = remaining / ground_vx;
                if (time_to_boundary < dt)
                    dt = time_to_boundary;
            }

            static inline double accepted_factor(double error_norm, bool was_rejected)
            {
                double factor = (error_norm == 0.0)
                                    ? kMaxFactor
                                    : std::min(kMaxFactor, kSafety * std::pow(error_norm, kErrorExponent));
                if (was_rejected)
                    factor = std::min(1.0, factor);
                return factor;
            }

            static inline double rejected_factor(double error_norm)
            {
                return std::max(kMinFactor, kSafety * std::pow(error_norm, kErrorExponent));
            }
        };
    } // namespace

    void BCLIBC_integrateTsitouras(
        BCLIBC_BaseEngine &eng,
        BCLIBC_BaseTrajDataHandlerInterface &handler,
        BCLIBC_TerminationReason &reason)
    {
        integrate_embedded_rk45<Tsitouras54Tableau, TsitourasController>(eng, handler, reason);
    }

    void BCLIBC_tsitourasGetStats(int &out_accepted, int &out_rejected)
    {
        embeddedRKGetStats<Tsitouras54Tableau, TsitourasController>(out_accepted, out_rejected);
    }

    void BCLIBC_tsitourasSetRelativeTolerance(double tolerance)
    {
        embeddedRKSetRelativeTolerance<Tsitouras54Tableau, TsitourasController>(tolerance);
    }

    void BCLIBC_tsitourasSetAbsoluteTolerance(double tolerance)
    {
        embeddedRKSetAbsoluteTolerance<Tsitouras54Tableau, TsitourasController>(tolerance);
    }

}; // namespace bclibc
