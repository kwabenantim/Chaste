/*

Copyright (c) 2005-2025, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#ifndef UBLASINCLUDES_HPP_
#define UBLASINCLUDES_HPP_

/**
 * @file
 * Convenience header for including ublas types in default namespace
 */

#include "ChastePragmas.hpp"

#include "UblasVectorInclude.hpp"
#include "UblasMatrixInclude.hpp"

CHASTE_DISABLE_BOOST_DEPRECATION_WARNING_BEGIN
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>
#include <boost/numeric/ublas/io.hpp>
CHASTE_DISABLE_BOOST_DEPRECATION_WARNING_END

namespace ublas = boost::numeric::ublas;

using boost::numeric::ublas::matrix_column;
using boost::numeric::ublas::matrix_row;
using boost::numeric::ublas::matrix_slice;

using boost::numeric::ublas::noalias;
using boost::numeric::ublas::prod;
using boost::numeric::ublas::outer_prod;
using boost::numeric::ublas::inner_prod;
using boost::numeric::ublas::trans;
using boost::numeric::ublas::norm_2;
using boost::numeric::ublas::slice;

#endif /*UBLASINCLUDES_HPP_*/
