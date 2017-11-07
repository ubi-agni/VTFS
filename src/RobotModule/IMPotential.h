/*
 * IMPotential.h
 *
 *  Created on: Oct 13, 2017
 *      Author: Qiang Li
 */

#pragma once

#include <cbf/potential.h>
#include <stdexcept>
#include <boost/shared_ptr.hpp>



class IMPotential : public CBF::Potential
{
protected:
  CBF::Float m_MaxGradientStep;
  CBF::Float m_Coefficient;
public:

  IMPotential(CBF::Float coefficient, CBF::Float max_gradient_step = 1.0):
    m_MaxGradientStep(max_gradient_step),
    m_Coefficient(coefficient)
  {
    if (m_MaxGradientStep < 0.0) throw std::runtime_error("negative norm");
  };


  virtual
  ~IMPotential() { }

	virtual CBF::Float norm(const CBF::FloatVector &v) { return v.norm(); }

  virtual CBF::Float distance(const CBF::FloatVector &v1, const CBF::FloatVector &v2) { return norm (v1 - v2); }

  virtual void gradient (
      CBF::FloatVector &result,
      const std::vector<CBF::FloatVector > &references,
      const CBF::FloatVector &input
  ) {
    //! First we find the closest reference vector (code seems never used)
    
    CBF::Float min_dist = std::numeric_limits<CBF::Float>::max();
    unsigned int min_index;
    min_index=0;
    //std::cout  << "[SquarePotential]: sizes: " << references[0].size() << " " << input.size() << std::endl;

    for (unsigned int i = 0; i < references.size(); ++i) {
      CBF::Float dist = distance(input, references[i]);
      if (dist < min_dist) {
        min_index = i;
        min_dist = dist;
      }
      else {
         min_index = 0;
      }

    }
    CBF_DEBUG("min_index " << min_index);
    

    //! The gradient of a square function is just negative of input - reference..
    result.resize(input.size());
    for (unsigned int i = 0; i < input.size(); ++i) {
      result[i] = 2.0*(references[min_index][i]-input[i]);
      result[i] *= m_Coefficient;
    }

    CBF::Float result_norm = norm(result);

    //! Normalize gradient step so it's not bigger than m_MaxGradientStep
    if (result_norm >= m_MaxGradientStep)
      result = (m_MaxGradientStep/result_norm) * result;

    std::cout<<"gradient from internal movement ";
    for (unsigned int i = 0; i < input.size(); ++i) {
        std::cout<<result[i]<<",";
    }
    std::cout<<std::endl;
  }

  virtual unsigned int dim() const { return 7; }
};

typedef boost::shared_ptr<IMPotential> IMPotentialPtr;

