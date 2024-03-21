// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_KRM_BLOCK_H
#define CH_KRM_BLOCK_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Sparse blocks loaded into the KRM global matrix, associated with a set of variables.
///
/// See ChSystemDescriptor for more information about the overall problem and data representation.
///
/// Notes:
/// - KRM blocks often have a physical interpretation as stiffness or damping, but not always, for example they can also
/// represent Hessians.
/// - KRM blocks, together with all masses and constraint Jacobians, are not always assembled in a system-level matrix.
/// That is usually done only when using direct sparse solvers or else for debugging/reporting purposes.
class ChApi ChKRMBlock {
  public:
    ChKRMBlock() {}
    ChKRMBlock(std::vector<ChVariables*> mvariables);
    ChKRMBlock(ChVariables* mvariableA, ChVariables* mvariableB);
    virtual ~ChKRMBlock() {}

    /// Assignment operator: copy from other object.
    ChKRMBlock& operator=(const ChKRMBlock& other);

    /// Set references to the constrained objects, each of ChVariables type.
    /// This automatically creates and resizes the KRM matrix, as needed.
    void SetVariables(std::vector<ChVariables*> mvariables);

    /// Returns the number of referenced ChVariables items
    size_t GetNumVariables() const { return variables.size(); }

    /// Access the m-th vector variable object
    ChVariables* GetVariable(unsigned int m) const { return variables[m]; }

    /// Access the KRM matrix as a single block, corresponding to the referenced ChVariable objects.
    ChMatrixRef GetMatrix() { return KRM; }

    /// Computes the product of the corresponding blocks in the system matrix (ie. the K matrix blocks) by 'vect', and
    /// add to 'result'.
    /// NOTE: the 'vect' and 'result' vectors must already have the size of the total variables & constraints in the
    /// system; the procedure will use the ChVariable offsets (that must be already updated) to know the indexes in
    /// result and vect.
    void MultiplyAndAdd(ChVectorRef result, ChVectorConstRef vect) const;

    /// Add the diagonal of the stiffness matrix block(s) as a column vector to 'result'.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie the size of the total variables &
    /// constraints in the system; the procedure will use the ChVariable offsets (that must be already updated).
    void DiagonalAdd(ChVectorRef result);

    /// Write the KRM matrix into the specified global matrix at the offsets of the referenced ChVariable objects.
    /// Assmebling the system-level sparse matrix is required only if using a direct sparse solver or for
    /// debugging/reporting purposes.
    void PasteInto(ChSparseMatrix& storage, bool add);

  private:
    ChMatrixDynamic<double> KRM;
    std::vector<ChVariables*> variables;
};

}  // end namespace chrono

#endif