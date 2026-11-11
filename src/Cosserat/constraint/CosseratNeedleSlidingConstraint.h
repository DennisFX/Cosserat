/******************************************************************************
 *               SOFA, Simulation Open-Framework Architecture                  *
 *                (c) 2020 INRIA, USTL, UJF, CNRS, MGH                    *
 *                                                                             *
 * This library is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 2.1 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This library is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this library; if not, write to the Free Software Foundation,     *
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
 *******************************************************************************
 *                      Plugin Cosserat v1.0                                   *
 *                                                                             *
 * This plugin is also distributed under the GNU LGPL (Lesser General          *
 * Public License) license with the same conditions than SOFA.                 *
 *                                                                             *
 * Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
 *               Ecole Centrale de Lille)                                      *
 *                                                                             *
 * Contact information: https://project.inria.fr/softrobot/contact/            *
 *                     adagolodjo@protonmail.com                               *
 ******************************************************************************/

#pragma once

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/behavior/Constraint.h>
#include <sofa/core/behavior/ConstraintResolution.h>

#include <algorithm>
#include <cmath>

namespace sofa::component::constraintset
{

    using sofa::core::ConstraintParams;
    using sofa::core::VecCoordId;
    using sofa::core::behavior::Constraint;
    using sofa::core::objectmodel::Data;
    using sofa::core::visual::VisualParams;
    using sofa::helper::ReadAccessor;
    using sofa::linearalgebra::BaseVector;
    using sofa::core::MultiVecDerivId ;

    /**
     * Gauss-Seidel resolution of one gel point with shaft friction: nbLateral sideways lines
     * (bilateral: the point stays on the needle axis) followed by one line along the needle
     * (Coulomb stick-slip). Along the needle the point sticks to the shaft until holding it
     * there would take more than
     *     limit + mu * |sideways force|,
     * then slides with that force opposing the slip. Forces are in the solver's units
     * (impulses, force * dt), and so is limit.
     */
    class NeedleFrictionResolution : public sofa::core::behavior::ConstraintResolution
    {
    public:
        /// nbLateral: 0, 1 or 2 sideways lines before the axial one
        NeedleFrictionResolution(unsigned int nbLateral, SReal limit, SReal mu)
            : ConstraintResolution(nbLateral + 1),
              m_nbLateral(nbLateral),
              m_limit(std::max(limit, SReal(0))),
              m_mu(std::max(mu, SReal(0)))
        {}

        void init(int line, SReal **w, SReal * /*force*/) override
        {
            // Inverse of the sideways block of the compliance matrix
            m_invW[0][0] = m_invW[0][1] = m_invW[1][0] = m_invW[1][1] = 0;
            if (m_nbLateral == 2)
            {
                const SReal a = w[line][line], b = w[line][line + 1];
                const SReal c = w[line + 1][line], e = w[line + 1][line + 1];
                const SReal det = a * e - b * c;
                if (std::abs(det) > SReal(1e-12) * std::abs(a * e))
                {
                    m_invW[0][0] = e / det;
                    m_invW[0][1] = -b / det;
                    m_invW[1][0] = -c / det;
                    m_invW[1][1] = a / det;
                    return;
                }
            }
            for (unsigned int k = 0; k < m_nbLateral; k++)
                if (w[line + k][line + k] != 0)
                    m_invW[k][k] = 1 / w[line + k][line + k];
        }

        void resolution(int line, SReal **w, SReal *d, SReal *force, SReal * /*dFree*/) override
        {
            const int axial = line + static_cast<int>(m_nbLateral);

            // 1. Sideways: the force that puts the point back on the needle axis
            SReal delta[2] = {0, 0};
            for (unsigned int a = 0; a < m_nbLateral; a++)
                for (unsigned int b = 0; b < m_nbLateral; b++)
                    delta[a] -= m_invW[a][b] * d[line + b];

            SReal dAxial = d[axial];
            SReal lateral2 = 0;
            for (unsigned int a = 0; a < m_nbLateral; a++)
            {
                force[line + a] += delta[a];
                dAxial += w[axial][line + a] * delta[a];  // its effect on the slip
                lateral2 += force[line + a] * force[line + a];
            }

            // 2. Along the needle: the force that stops the slip (stick), at most the limit (slip)
            const SReal wAxial = w[axial][axial];
            const SReal stick = (wAxial > 0) ? force[axial] - dAxial / wAxial : SReal(0);
            const SReal bound = m_limit + m_mu * std::sqrt(lateral2);
            force[axial] = std::clamp(stick, -bound, bound);
        }

    private:
        unsigned int m_nbLateral;
        SReal m_limit;
        SReal m_mu;
        SReal m_invW[2][2]{};
    };

    /**
     * Holds the gel points (the constraint points in the FEM) on the needle. Each point gets
     * one line per sideways direction (needle-local y and z), and, with friction, a third line
     * along the needle: lines are [y, z] or [y, z, axial] per point, in point order.
     */
    template <class DataTypes>
    class CosseratNeedleSlidingConstraint : public Constraint<DataTypes>
    {
    public:
        SOFA_CLASS(SOFA_TEMPLATE(CosseratNeedleSlidingConstraint, DataTypes), SOFA_TEMPLATE(Constraint, DataTypes));
        typedef typename DataTypes::VecCoord VecCoord;
        typedef typename DataTypes::VecDeriv VecDeriv;
        typedef typename DataTypes::Coord Coord;
        typedef typename DataTypes::Deriv Deriv;
        typedef typename DataTypes::MatrixDeriv MatrixDeriv;
        typedef typename Coord::value_type Real;
        typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

        typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
        typedef Data<VecCoord> DataVecCoord;
        typedef Data<VecDeriv> DataVecDeriv;
        typedef Data<MatrixDeriv> DataMatrixDeriv;
        typedef type::vector<unsigned int> SetIndexArray;

    public:
        CosseratNeedleSlidingConstraint(MechanicalState *object = nullptr);

        ~CosseratNeedleSlidingConstraint() override;

        ////////////////////////// Inherited from BaseObject ////////////////////
        void init() override;
        void reinit() override;
        void draw(const VisualParams *vparams) override;
        /////////////////////////////////////////////////////////////////////////

        ////////////////////////// Inherited from Actuator //////////////////////
        void buildConstraintMatrix(const ConstraintParams *cParams,
                                   DataMatrixDeriv &cMatrix,
                                   unsigned int &cIndex,
                                   const DataVecCoord &x) override;
        void getConstraintViolation(const ConstraintParams *cParams,
                                    BaseVector *resV, const DataVecCoord &x, const DataVecDeriv &v) override;
        //    void getConstraintViolation(const ConstraintParams* cParams,
        //                                BaseVector *resV, const DataVecCoord &x, const DataVecDeriv &v) override;
        void getConstraintResolution(const ConstraintParams *,
                                     std::vector<core::behavior::ConstraintResolution *> &resTab,
                                     unsigned int &offset) override;

        /////////////////////////////////////////////////////////////////////////

        ////////////////////////// Inherited from BaseConstraint ////////////////
        //    void storeLambda(const ConstraintParams* cParams,
        //                     core::MultiVecDerivId res,
        //                     const BaseVector* lambda) override;
        /////////////////////////////////////////////////////////////////////////

        void storeLambda(const ConstraintParams */*cParams*/, Data<VecDeriv> &result, const Data<MatrixDeriv> &jacobian, const sofa::linearalgebra::BaseVector *lambda)
        {
            auto res = sofa::helper::getWriteAccessor(result);
            const MatrixDeriv &j = jacobian.getValue();
            j.multTransposeBaseVector(res, lambda); // lambda is a vector of scalar value so block size is one.
        }


         void storeLambda(const ConstraintParams *cParams, MultiVecDerivId res, const sofa::linearalgebra::BaseVector *lambda) override
         {
            SOFA_UNUSED(cParams);
            SOFA_UNUSED(res);
            SOFA_UNUSED(lambda);
         }

    protected:
        // Input data
        Data<type::vector<Real>> d_value;
        Data<unsigned int> d_valueIndex;
        Data<helper::OptionsGroup> d_valueType;
        Data<type::Vec<3, bool>> d_useDirections;
        Data<type::vector<Real>> d_friction;
        Data<Real> d_frictionCoefficient;
        // displacement = the constraint will impose the displacement provided in data d_inputValue[d_iputIndex]
        // force = the constraint will impose the force provided in data d_inputValue[d_iputIndex]

    protected:
        ////////////////////////// Inherited attributes ////////////////////////////
        /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
        /// Bring inherited attributes and function in the current lookup context.
        /// otherwise any access to the base::attribute would require
        /// the "this->" approach.
        using Constraint<DataTypes>::d_componentState;
        ////////////////////////////////////////////////////////////////////////////
        /// \brief internalInit
        unsigned m_nbLines{} ;
        using Constraint<DataTypes>::d_constraintIndex;

        void internalInit();

        /// True when each point gets a line along the needle (friction or frictionCoefficient set)
        bool hasAxialLine() const;
        /// Friction limit of point i, in scene force units
        Real frictionLimit(unsigned int i) const;
    };

    // Declares template as extern to avoid the code generation of the template for
    // each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
    // extern template class CosseratNeedleSlidingConstraint<defaulttype::Vec3Types>;

} // namespace sofa
