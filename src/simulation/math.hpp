#ifndef SIMULATION_MATH_HPP
#define SIMULATION_MATH_HPP

#include <cstring>
#include <cstdint>
#include <cassert>
#include <cstdlib>
#include <utility>
#include <cstdio>
#include <limits>
#include <cmath>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

namespace math {
    /** Implementation of "maxstep" of Ref. 2.*/
    void maxstep(
            double *s, uint32_t *j, const double *fvec, const double *avec,
            const double *fvec_delta, const double *avec_delta,
            const bool *c, const bool *nc, uint32_t n, uint32_t d
    );

    /** Implementation of "drive_to_zero" of Ref. 2.*/
    void fdirection(double *fvec_delta, const double *amat, const bool *c, uint32_t n, uint32_t d);

    /** Implementation of "drive_to_zero" of Ref. 2.*/
    void drive_to_zero(
            const double *amat, double *avec, double *fvec, bool *c, bool *nc, uint32_t n, uint32_t d
    );

    /** Implementation of the algorithm described in Ref. 2.*/
    void qp_solve(const double *amat, const double *bvec, double *fvec, uint32_t n);

    /** Solves for a symmetric, positive semi definite matrix. */
    void lp_solve(double *amat, double *xvec, double *bvec, uint32_t n);

    /**
     * Helper functions.
     */

    /** res = mat * vec */
    void mat_mul_vec(double *res, const double *mat, const double *vec, uint32_t n);

    /** r += v */
    void vec_add_equal(uint32_t n, double *r, const double *v);

    /** r -= v */
    void vec_sub_equal(uint32_t n, double *r, const double *v);

    /** v1 = v2 */
    void vec_assign(uint32_t n, double *v1, const double *v2);

    /** v *= s */
    void vec_mul_scalar(uint32_t n, double *v, double s);

    /** dot product of v1 and v2 */
    double vec_dot(uint32_t n, const double *v1, const double *v2);

    /** length squared */
    double vec_len_squared(uint32_t n, double *v);
}

#endif //SIMULATION_MATH_HPP
