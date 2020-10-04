#include "math.hpp"

void math::maxstep(
        double *s, uint32_t *j, const double *fvec, const double *avec,
        const double *fvec_delta, const double *avec_delta,
        const bool *c, const bool *nc, uint32_t n, uint32_t d)
{
    *s = std::numeric_limits<double>::max();
    *j = -1;

    if (avec_delta[d] > 0.) {         // if Delta a_d > 0
        *j = d;                        // j = d
        *s = -avec[d] / avec_delta[d]; // s = -a_d / Delta a_d
    }

    for (uint32_t i = 0; i < n; i++) {
        if (c[i] && fvec_delta[i] < 0.) {            // if Delta f_d < 0
            double s_prime = -fvec[i] / fvec_delta[i]; // s' = -f_i / Delta f_i
            if (s_prime < *s) {                       // if s' < s
                *s = s_prime;                         // s = s'
                *j = i;                               // j = i
            }
        }
    }

    for (uint32_t i = 0; i < n; i++) {
        if (nc[i] && avec_delta[i] < 0.) {           // if Delta a_i < 0
            double s_prime = -avec[i] / avec_delta[i]; // s' = -a_i / Delta a_i
            if (s_prime < *s) {                       // if s' < s
                *s = s_prime;                         // s = s'
                *j = i;                               // j = i
            }
        }
    }

    if (*j == (uint32_t) -1 || (*s == 0. && *j != d) || *s == std::numeric_limits<double>::max()) {
        assert(0); // assert that a value for j is chosen
    }
}

void math::fdirection(double *fvec_delta, const double *amat, const bool *c, uint32_t n, uint32_t d)
{
    memset(fvec_delta, 0., n * sizeof(double)); // Delta f = 0
    fvec_delta[d] = 1.;                         // Delta f_d = 1

    uint32_t c_count = 0; // the amount of indices in C
    for (uint32_t i = 0; i < n; i++) {
        if (c[i]) c_count++;
    }

    if (c_count == 0) return; // no work to be done

    // A_11 = A_CC
    auto amat_11 = (double *) malloc(c_count * c_count * sizeof(double));
    uint32_t amat_11_index = 0;
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            if (c[i] && c[j]) amat_11[amat_11_index++] = amat[i * n + j];
        }
    }

    assert(amat_11_index == c_count * c_count);

    // -v_1 = -A_Cd
    auto vvec_1 = (double *) malloc(c_count * sizeof(double));
    uint32_t vvec_1_index = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (c[i]) vvec_1[vvec_1_index++] = -amat[i * n + d];
    }

    assert(vvec_1_index == c_count);

    // solve A_11 * x = -v_1
    auto x = (double *) malloc(c_count * sizeof(double));
    lp_solve(amat_11, x, vvec_1, c_count);

    // transfer x into Delta f
    uint32_t x_index = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (c[i]) fvec_delta[i] = x[x_index++];
    }

    free(x);
    free(vvec_1);
    free(amat_11);
}

void math::drive_to_zero(
        const double *amat, double *avec, double *fvec, bool *c, bool *nc, uint32_t n, uint32_t d)
{
    auto fvec_delta = (double *) malloc(n * sizeof(double));
    auto avec_delta = (double *) malloc(n * sizeof(double));
    double s;
    uint32_t j;

    l1:
    fdirection(fvec_delta, amat, c, n, d);        // Delta f = fdirection(d)

    for (uint32_t i = 0; i < n; i++) {
        if (c[i] && fvec[i] == 0. && fvec_delta[i] < 0.) {
            assert(0); // todo debug, in a later stage this will result in a crash
        }
    }

    mat_mul_vec(avec_delta, amat, fvec_delta, n); // Delta a = A * Delta f

    for (uint32_t i = 0; i < n; i++) {
        if (nc[i] && avec[i] == 0. && avec_delta[i] < 0.) {
            assert(0); // todo debug, in a later stage this will result in a crash
        }
    }
    // (s, j) = maxstep(f, a, Delta f, Delta a, d)
    maxstep(&s, &j, fvec, avec, fvec_delta, avec_delta, c, nc, n, d);

    vec_mul_scalar(n, fvec_delta, s);   // Delta f *= s
    vec_add_equal(n, fvec, fvec_delta); // f += Delta f

    vec_mul_scalar(n, avec_delta, s);   // Delta a += s
    vec_add_equal(n, avec, avec_delta); // a += Delta a

    if (c[j]) {         // if j in C
        c[j] = false;   // C = C - {j}
        nc[j] = true;   // NC = NC U {j}
        goto l1;
    } else if (nc[j]) { // else if j in NC
        nc[j] = false;  // NC = NC - {j}
        c[j] = true;    // C = C U {j}
        goto l1;
    } else { // j must be d, implying a_d = 0
        c[j] = true;    // C = C U {j}
    }

    free(avec_delta);
    free(fvec_delta);
}

void math::qp_solve(const double *amat, const double *bvec, double *fvec, uint32_t n)
{
    memset(fvec, 0., n * sizeof(double));       // f = 0
    auto avec = (double *) malloc(n * sizeof(double));
    memcpy(avec, bvec, n * sizeof(double));      // a = b

    auto c = (bool *) calloc(n, sizeof(bool));  // C = emptyset
    auto nc = (bool *) calloc(n, sizeof(bool)); // NC = emptyset

    // while exists d such that a_d < 0
    bool done;
    do {
        done = true;
        for (uint32_t d = 0; d < n; d++) {
            // threshold is needed since drive_to_zero can bring avec[d] to a really
            // small negative number but not not zero
            const double THRESHOLD = -1.e-14;
            if (avec[d] < THRESHOLD) {
                // drive-to-zero(d)
                drive_to_zero(amat, avec, fvec, c, nc, n, d);
                if (avec[d] < THRESHOLD) {
                    assert(0);
                }
                done = false;
                break;
            }
        }
    } while (!done);

    free(nc);
    free(c);
    free(avec);
}

void math::mat_mul_vec(double *res, const double *mat, const double *vec, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        res[i] = 0.;
        for (uint32_t j = 0; j < n; j++) {
            res[i] += mat[i * n + j] * vec[j];
        }
    }
}

void math::vec_add_equal(uint32_t n, double *r, const double *v)
{
    for (uint32_t i = 0; i < n; i++) {
        r[i] += v[i];
    }
}

void math::vec_sub_equal(uint32_t n, double *r, const double *v)
{
    for (uint32_t i = 0; i < n; i++) {
        r[i] -= v[i];
    }
}

void math::vec_assign(uint32_t n, double *v1, const double *v2)
{
    memcpy(v1, v2, n * sizeof(double));
}

void math::vec_mul_scalar(uint32_t n, double *v, double s)
{
    for (uint32_t i = 0; i < n; i++) {
        v[i] *= s;
    }
}

double math::vec_dot(uint32_t n, const double *v1, const double *v2)
{
    double dot = 0.f;
    for (uint32_t i = 0; i < n; i++) {
        dot += v1[i] * v2[i];
    }

    return dot;
}

double math::vec_len_squared(uint32_t n, double *v)
{
    return vec_dot(n, v, v);
}

void math::lp_solve(double *amat, double *xvec, double *bvec, uint32_t n)
{
    gsl_matrix *a = gsl_matrix_alloc(n, n);
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            gsl_matrix_set(a, i, j, amat[i * n + j]);
        }
    }
    gsl_permutation *p = gsl_permutation_alloc(n);
    gsl_linalg_pcholesky_decomp(a, p);
    gsl_vector *x = gsl_vector_alloc(n);
    for (uint32_t i = 0; i < n; i++) {
        gsl_vector_set(x, i, bvec[i]);
    }
    gsl_linalg_pcholesky_svx(a, p, x);
    for (uint32_t i = 0; i < n; i++) {
        xvec[i] = gsl_vector_get(x, i);
    }
    gsl_vector_free(x);
    gsl_permutation_free(p);
    gsl_matrix_free(a);
}