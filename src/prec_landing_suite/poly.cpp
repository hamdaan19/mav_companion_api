#include <stdio.h>
#include <iostream>
#include <gsl/gsl_poly.h>

int
main (void)
{
//   int i;
//   /* coefficients of P(x) =  -1 + x^5  */
//   double a[6] = { -1, 0, 0, 0, 0, 1 };
//   double z[10];

//   gsl_poly_complex_workspace * w
//       = gsl_poly_complex_workspace_alloc (6);

//   gsl_poly_complex_solve (a, 6, w, z);

//   gsl_poly_complex_workspace_free (w);

//   for (i = 0; i < 5; i++)
//     {
//       printf ("z%d = %+.18f %+.18f\n",
//               i, z[2*i], z[2*i+1]);
//     }

double c[4] = {3.87021, -3.98185, 12.5298, 2.66052};
double c_[4];
for (int i = 0; i < 4; i++){
    c_[i] = c[i]/c[0];
    std::cout << c_[i] << std::endl;
}

double roots[3];

int i = gsl_poly_solve_cubic(c_[1], c_[2], c_[3], &roots[0], &roots[1], &roots[2]);

double d = gsl_poly_eval(c, 4, 0.5);

std::cout << roots[0] << std::endl; 

  return 0;
}