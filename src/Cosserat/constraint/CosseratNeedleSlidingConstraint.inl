/******************************************************************************
 *               SOFA, Simulation Open-Framework Architecture                  *
 *                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
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
 *                           Plugin Cosserat  v1.0                             *
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

#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/Vec.h>
#include <sofa/component/constraint/lagrangian/model/BilateralLagrangianConstraint.h>
#include "CosseratNeedleSlidingConstraint.h"

namespace sofa::component::constraintset
{

  using sofa::core::objectmodel::ComponentState;
  using sofa::helper::WriteAccessor;

  using sofa::core::objectmodel::ComponentState;
  using sofa::core::visual::VisualParams;
  using sofa::linearalgebra::BaseVector;
  using sofa::helper::ReadAccessor;
  using sofa::type::Vec4f;
  using sofa::type::Vec3;
  using sofa::type::vector;
  using sofa::helper::OptionsGroup;
  using sofa::component::constraint::lagrangian::model::BilateralConstraintResolution;

  template <class DataTypes>
  CosseratNeedleSlidingConstraint<DataTypes>::CosseratNeedleSlidingConstraint(MechanicalState *object)
      : Inherit1(object),
        d_value(initData(&d_value, "value", "Displacement or force to impose.\n")),
        d_valueIndex(initData(&d_valueIndex, (unsigned int)0, "valueIndex",
                              "Index of the value (in InputValue vector) that we want to impose \n"
                              "If unspecified the default value is {0}")),
        d_valueType(initData(&d_valueType, {"displacement", "force"}, "valueType",
                             "displacement = the contstraint will impose the displacement provided in data value[valueIndex] \n"
                             "force = the contstraint will impose the force provided in data value[valueIndex] \n"
                             "If unspecified, the default value is displacement")),
        d_useDirections(initData(&d_useDirections, type::Vec<3, bool>(0, 1, 1), "useDirections",
                                 "Sideways directions to constrain (needle-local y and z). The first entry\n"
                                 "is ignored: the direction along the needle is constrained by friction.\n")),
        d_friction(initData(&d_friction, "friction",
                            "Friction along the needle at each gel point: the largest force (scene units)\n"
                            "that keeps the point stuck to the shaft before it slides. One value for all\n"
                            "points, or one per point (points past the end of the list get none).\n"
                            "Empty (the default) with frictionCoefficient 0: no friction.\n")),
        d_frictionCoefficient(initData(&d_frictionCoefficient, Real(0), "frictionCoefficient",
                                       "Coulomb coefficient: adds frictionCoefficient * |sideways force| at\n"
                                       "the point to its friction limit.\n"))
  {}

  template <class DataTypes>
  CosseratNeedleSlidingConstraint<DataTypes>::~CosseratNeedleSlidingConstraint()
  = default;

  template <class DataTypes>
  void CosseratNeedleSlidingConstraint<DataTypes>::init()
  {
    Inherit1::init();
    d_componentState.setValue(ComponentState::Valid);

    internalInit();
  }

  template <class DataTypes>
  void CosseratNeedleSlidingConstraint<DataTypes>::reinit()
  {
    internalInit();
  }

  template <class DataTypes>
  void CosseratNeedleSlidingConstraint<DataTypes>::internalInit()
  { 
    if (d_value.getValue().size() == 0)
    {
      WriteAccessor<Data<vector<Real>>> value = d_value;
      value.resize(1);
      value[0] = 0.;
    }

    // check for errors in the initialization
    if (d_value.getValue().size() < d_valueIndex.getValue())
    {
      msg_warning() << "Bad size for data value (size=" << d_value.getValue().size() << "), or wrong value for data valueIndex (valueIndex=" << d_valueIndex.getValue() << "). Set default valueIndex=0.";
      d_valueIndex.setValue(0);
    }
    
  }

  template <class DataTypes>
  bool CosseratNeedleSlidingConstraint<DataTypes>::hasAxialLine() const
  {
    return !d_friction.getValue().empty() || d_frictionCoefficient.getValue() > 0;
  }

  template <class DataTypes>
  typename CosseratNeedleSlidingConstraint<DataTypes>::Real
  CosseratNeedleSlidingConstraint<DataTypes>::frictionLimit(unsigned int i) const
  {
    const auto &friction = d_friction.getValue();
    if (friction.empty())
      return 0;
    const Real limit = (friction.size() == 1) ? friction[0] : (i < friction.size() ? friction[i] : 0);
    return std::max(limit, Real(0));
  }

  template <class DataTypes>
  void CosseratNeedleSlidingConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams *cParams, DataMatrixDeriv &cMatrix, unsigned int &cIndex, const DataVecCoord &x)
  {
    if (d_componentState.getValue() != ComponentState::Valid)
      return;

    SOFA_UNUSED(cParams);
    MatrixDeriv &matrix = *cMatrix.beginEdit();
    const VecCoord &positions = x.getValue();
    d_constraintIndex.setValue(cIndex);

    const type::Vec<3, bool> use = d_useDirections.getValue();
    const bool axial = hasAxialLine();

    // Lines per point: [y, z] or [y, z, axial]. The mapped point is
    // (0, t1.(P - proj), t2.(P - proj)), so y and z are its distance to the needle axis; x
    // is always 0, but its velocity is the gel point's slip along the needle.
    for (unsigned int i = 0; i < positions.size(); i++)
    {
      if (use[1])
      {
        MatrixDerivRowIterator c_it = matrix.writeLine(cIndex++);
        c_it.addCol(i, Coord(0, 1, 0));
      }
      if (use[2])
      {
        MatrixDerivRowIterator c_it = matrix.writeLine(cIndex++);
        c_it.addCol(i, Coord(0, 0, 1));
      }
      if (axial)
      {
        MatrixDerivRowIterator c_it = matrix.writeLine(cIndex++);
        c_it.addCol(i, Coord(1, 0, 0));
      }
    }
    cMatrix.endEdit();
    m_nbLines = cIndex - d_constraintIndex.getValue();
  }

  template <class DataTypes>
  void CosseratNeedleSlidingConstraint<DataTypes>::getConstraintViolation(const ConstraintParams *cParams,
                                                                          BaseVector *resV, const DataVecCoord &x, const DataVecDeriv &v)
  {
    if (d_componentState.getValue() != ComponentState::Valid)
      return;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(v);
    // x is the free position: the position at the start of the step plus the free motion.
    // Along the needle that is 0 + (slip velocity) * dt, the slip the step would make
    // without friction.
    const VecCoord &positions = x.getValue();
    const type::Vec<3, bool> use = d_useDirections.getValue();
    const bool axial = hasAxialLine();

    unsigned int line = d_constraintIndex.getValue();
    for (unsigned int i = 0; i < positions.size(); i++)
    {
      if (use[1])
        resV->set(line++, positions[i][1]);
      if (use[2])
        resV->set(line++, positions[i][2]);
      if (axial)
        resV->set(line++, positions[i][0]);
    }
  }

  template <class DataTypes>
  void CosseratNeedleSlidingConstraint<DataTypes>::getConstraintResolution(const ConstraintParams *,
                                                                           std::vector<core::behavior::ConstraintResolution *> &resTab,
                                                                           unsigned int &offset)
  {
    ReadAccessor<Data<VecCoord>> positions = this->mstate->readPositions();
    const type::Vec<3, bool> use = d_useDirections.getValue();

    if (!hasAxialLine())
    {
      for (size_t i = 0; i < positions.size(); i++)
      {
        if (use[1])
          resTab[offset++] = new BilateralConstraintResolution();
        if (use[2])
          resTab[offset++] = new BilateralConstraintResolution();
      }
      return;
    }

    // The solver's forces are impulses (force * dt), so the limits are too
    const SReal dt = this->getContext()->getDt();
    const unsigned int nbLateral = static_cast<unsigned int>(use[1]) + static_cast<unsigned int>(use[2]);
    const SReal mu = d_frictionCoefficient.getValue();
    for (size_t i = 0; i < positions.size(); i++)
    {
      resTab[offset] = new NeedleFrictionResolution(nbLateral, frictionLimit(i) * dt, mu);
      offset += nbLateral + 1;
    }
  }

  template <class DataTypes>
  void CosseratNeedleSlidingConstraint<DataTypes>::draw(const VisualParams *vparams)
  {
    SOFA_UNUSED(vparams);
    if (d_componentState.getValue() != ComponentState::Valid)
      return;
  }
} // namespace sofa::component::constraintset
