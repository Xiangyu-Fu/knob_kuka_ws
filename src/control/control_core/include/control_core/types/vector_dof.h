/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */


#ifndef CONTROL_CORE_VECTOR_DOF_H
#define CONTROL_CORE_VECTOR_DOF_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/vector_base.h>

namespace control_core
{
/*!
 * \brief The VectorDof class.
 *
 * The VectorDof is of type VectorRef.
 * This vector represents the robot's dof and 
 * has a fixed size that can not be changed
 */
template <typename _Scalar, int _Rows = Eigen::Dynamic>
class VectorDof :
  public Eigen::Matrix<_Scalar, _Rows, 1>,
  public VectorBase<VectorDof<_Scalar, _Rows> >
{

public:
  enum
  {
      RowsAtCompileTime = _Rows,
  };

  typedef _Scalar Scalar;
  typedef Eigen::Matrix<_Scalar,RowsAtCompileTime,1> Base;
  typedef VectorBase<VectorDof<Scalar, RowsAtCompileTime> > VBase;

protected:

public:
  /*!
   * \brief Default Constructor.
   */
  VectorDof() :
    Base()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  VectorDof(const VectorDof& other) :
    Base(other)
  {
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template<typename OtherDerived>
  VectorDof(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of VBase class.
   */
  using Base::operator=;
  using VBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  VectorDof& operator=(const VectorDof& other)
  {
    Base::operator=(other);
    return *this;
  }

private:

  /*!
   * \brief Don't allow resize.
   */
  using Base::resize;
  using Base::resizeLike;
  using Base::conservativeResize;

  /*!
   * \brief For now not implemented.
   */
  using Base::LinSpaced;

};

}  // namespace control_core


#endif  // CONTROL_CORE_VECTOR_DOF_H
