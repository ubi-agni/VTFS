/*
 * WuPotential.h
 *
 *  Created on: Jul 27, 2010
 *      Author: mschoepf
 */

#pragma once

#include <cbf/potential.h>
#include <stdexcept>
#include <boost/shared_ptr.hpp>

class WuPotential : public CBF::Potential
{
protected:
  CBF::FloatVector mins, maxs;
  CBF::Float m_MaxGradientStep;
  CBF::Float m_Coefficient;
public:

  WuPotential(CBF::FloatVector mins, CBF::FloatVector maxs, CBF::Float coefficient, CBF::Float max_gradient_step = 1.0):
    mins (mins),
    maxs (maxs),
    m_MaxGradientStep(max_gradient_step),
    m_Coefficient(coefficient)
  {
    if (mins.size() != maxs.size()) throw std::runtime_error("mismatching dimensions");
    if (m_MaxGradientStep < 0.0) throw std::runtime_error("negative norm");
  };


  virtual
  ~WuPotential() { }

	virtual CBF::Float norm(const CBF::FloatVector &v) { return v.norm(); }

  virtual CBF::Float distance(const CBF::FloatVector &v1, const CBF::FloatVector &v2) { return norm (v1 - v2); }

  virtual void gradient (
      CBF::FloatVector &result,
      const std::vector<CBF::FloatVector > &references,
      const CBF::FloatVector &input
  ) {
    //! First we find the closest reference vector
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
    CBF_DEBUG("min_index " << min_index)

    //! The gradient of a square function is just negative of input - reference..
    result.resize(input.size());

    for (unsigned int i = 0; i < input.size(); ++i) {
      result[i] = -1.0*(pow(maxs[i] - mins[i], 2.0) * (2.0 * input[i] - maxs[i] - mins[i]))
                / (pow(maxs[i] - input[i], 2) * pow(input[i] - mins[i], 2));

      result[i] *= m_Coefficient;
    }
    //std::cout << "Input: " << input << std::endl;
    //std::cout << "Corrections: " << result << std::endl;
    CBF::Float result_norm = norm(result);
    // CBF_DEBUG("result_norm " << result_norm)

    //! Normalize gradient step so it's not bigger than m_MaxGradientStep
    if (result_norm >= m_MaxGradientStep)
      result = (m_MaxGradientStep/result_norm) * result;
    // std::cout << "[SquaredPotential]: result: " << result << std::endl;
  }

  virtual unsigned int dim() const { return maxs.size(); }
};

typedef boost::shared_ptr<WuPotential> WuPotentialPtr;

