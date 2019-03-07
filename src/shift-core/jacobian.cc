#include <assert.h>

#include "shift-core/context.h"
#include "shift-core/jacobian.h"

static inline double infinityNormB(int rank, int colDim, double* J) {
  double max = 0;
  for(int i = 0; i < rank; i++) {
    for(int j = rank; j < colDim; j++) {
      max = std::max(max, std::abs(J[i * colDim + j]));
    }
  }
  return max;
}

void Shift::Jacobian::initialize() {
  std::cout << "initializing Jacobian" << std::endl;
  setLookup();
  mallocEquations();
  compressLoopMembers();
  setPO1();
  translatePermutations();
  setBooleanPO2();
  evaluateSparsity();
  setPO2();
  factorPO2();
  setBMatrixSparse();
  setConstraintPointers();
  setSubstructurePointers();
}

void Shift::Jacobian::analyze() {
  setPO1();
  translatePermutations();
  setBooleanPO2();
  evaluateSparsity();
  setPO2();
  factorPO2();
  setBMatrixSparse();
}

void Shift::Jacobian::setLookup() {

  assert(context_);

  jointLookup_.clear();
  jointLookup_.resize(context_->getActiveJointCount());

  dqLookup_.clear();
  dqLookup_.resize(context_->getActiveJointCount());

  qLookup_.clear();
  qLookup_.resize(context_->getActiveJointCount());

  dqLookupCompressed_.clear();
  dqLookupCompressed_.resize(context_->getNonRigidJointCount());

  std::cout << "setting lookup" << std::endl;

  int jointCount = 0;
  int dqCount = 0;
  int qCount = 0;
  unsigned int i = 0;
  for(auto& joint : context_->getActiveJoints()) {
    if(joint->getqCount() > 0) {
      jointLookup_[i] = jointCount;
      dqLookup_[i] = dqCount;
      qLookup_[i] = qCount;
      jointCount++;
    } else {
      jointLookup_[i] = -1;
      dqLookup_[i] = -1;
      qLookup_[i] = -1;
    }
    i++;
    dqCount += joint->getdqCount();
    qCount += joint->getqCount();
  }

  for(unsigned int j = 0; j < context_->getActiveJointCount(); j++) {
    if(jointLookup_[j] > -1) {
      dqLookupCompressed_[jointLookup_[j]] = dqLookup_[j];
    }
  }
}

void Shift::Jacobian::mallocEquations() {

  if(J_) {
    delete[] J_;
    J_ = NULL;
  }

  if(Jxx_) {
    delete[] Jxx_;
    Jxx_ = NULL;
  }

  if(Jb_) {
    delete[] Jb_;
    Jb_ = NULL;
  }

  if(cs_) {
    delete[] cs_;
    cs_ = NULL;
  }

  if(dcs_) {
    delete[] dcs_;
    dcs_ = NULL;
  }

  auto substructureIndices = context_->getSubstructureIndices();
  substructures_.clear();
  substructures_.resize(substructureIndices.size());

  // get total size of Jacobian
  jacobianSize_ = 0;
  constraintEquationCount_ = 0;
  unsigned int qOffset = 0;
  unsigned int dqOffset = 0;
  unsigned int jointOffset = 0;
  unsigned int jacobianSize = 0;
  unsigned int constraintEquationCount = 0;
  for(unsigned int i = 0; i < substructureIndices.size(); i++) {
    substructures_[i].setIndex(i);
    unsigned int qCount = 0;
    unsigned int dqCount = 0;
    unsigned int jointCount = 0;
    unsigned int chordConstraintCount = 0;
    for(unsigned int j = 0; j < substructureIndices[i].size(); j++) {
      /* iterate over loops in substructure */
      auto loopMembers = context_->getLoopMembers();
      for(unsigned int k = 1; k < loopMembers[j].size(); k++) {
        /* iterate over loop members in loop */
        auto joint = context_->getActiveJoints().at(loopMembers[j][k]);
        qCount += joint->getqCount();
        dqCount += joint->getdqCount();
        jointCount += (joint->getqCount() > 0) ? 1 : 0;
      }
      chordConstraintCount++;
    }
    substructures_[i].setqOffset(qOffset);
    substructures_[i].setdqOffset(dqOffset);
    substructures_[i].setJointOffset(jointOffset);
    qOffset += qCount;
    dqOffset += dqCount;
    jointOffset += jointCount;
    jacobianSize += chordConstraintCount * 6 * dqCount;
    constraintEquationCount += 6 * chordConstraintCount;
    substructures_[i].setqCount(qCount);
    substructures_[i].setdqCount(dqCount);
    substructures_[i].setJointCount(jointCount);
    substructures_[i].setChordConstraintCount(chordConstraintCount);
  }

  J_ = new double[jacobianSize]{};
  Jxx_ = new int[jacobianSize]{};
  Jb_ = new bool[jacobianSize]{};
  cs_ = new double[constraintEquationCount]{};
  dcs_ = new double[constraintEquationCount]{};

  jacobianSize = 0;
  unsigned int constraintEquationOffset = 0;

  for(auto& substructure : substructures_) {

    substructure.setJ(J_ + jacobianSize);
    substructure.setJxx(Jxx_ + jacobianSize);
    substructure.setJb(Jb_ + jacobianSize);

    jacobianSize +=
      substructure.getConstraintEquationCount() * substructure.getdqCount();

    substructure.setcs(cs_ + constraintEquationOffset);
    substructure.setdcs(dcs_ + constraintEquationOffset);

    constraintEquationOffset += substructure.getConstraintEquationCount();
  }

  isDependentPO1_.resize(dqOffset);
  isDependentPO3_.resize(dqOffset);

  pv_12_.resize(dqOffset);
  pv_21_.resize(dqOffset);
  pv_13_.resize(dqOffset);
  pv_31_.resize(dqOffset);
  pv_23_.resize(dqOffset);
  pv_32_.resize(dqOffset);

  //
  //   if(context->jointDOF) {
  //     free(context->jointDOF);
  //   }
  //   context->jointDOF = calloc(context->activeJointCount, sizeof(int));
  //   if(context->jointDOFCompressed) {
  //     free(context->jointDOFCompressed);
  //   }
  //   context->jointDOFCompressed =
  //     calloc(context->nonRigidJointCount, sizeof(int));
  //
  //
  //   for(int i = 0; i < context->activeJointCount; i++) {
  //     s2_body_t* childBody = context->bodyPointers[i];
  //
  //
  //     if(!(childBody->isVirtual || childBody->inertia == NULL ||
  //          childBody->inertia->m < 1e-9)) {
  //       if(context->jointLookup[i] > -1) {
  //         context->hasMass[context->jointLookup[i]] = true;
  //         context->hasMassdq[context->dqLookup[i]] = true;
  //
  //
  //         switch(context->jointPointers[i]->jointType) {
  //           case S2_JOINT_SPHERICAL: {
  //             context->hasMassdq[context->dqLookup[i] + 1] = true;
  //             context->hasMassdq[context->dqLookup[i] + 2] = true;
  //             break;
  //           }
  //           case S2_JOINT_FLOAT: {
  //             context->hasMassdq[context->dqLookup[i] + 1] = true;
  //             context->hasMassdq[context->dqLookup[i] + 2] = true;
  //             context->hasMassdq[context->dqLookup[i] + 3] = true;
  //             context->hasMassdq[context->dqLookup[i] + 4] = true;
  //             context->hasMassdq[context->dqLookup[i] + 5] = true;
  //             break;
  //           }
  //           default:
  //             break;
  //         }
  //       }
  //     }
  //
  //     switch(context->jointPointers[i]->jointType) {
  //       case S2_JOINT_RX:
  //       case S2_JOINT_RY:
  //       case S2_JOINT_RZ:
  //       case S2_JOINT_TX:
  //       case S2_JOINT_TY:
  //       case S2_JOINT_TZ: {
  //         context->jointDOF[i] = 1;
  //         break;
  //       }
  //       case S2_JOINT_SPHERICAL: {
  //         context->jointDOF[i] = 3;
  //         break;
  //       }
  //       case S2_JOINT_FLOAT: {
  //         context->jointDOF[i] = 6;
  //         break;
  //       }
  //       case S2_JOINT_RIGID: {
  //         context->jointDOF[i] = 0;
  //         break;
  //       }
  //       default:
  //         break;
  //     }
  //   }
  //
  //   /* create compressed parent child list */
  //   if(context->pclCompressed) {
  //     free(context->pclCompressed);
  //   }
  //   context->pclCompressed = calloc(context->nonRigidJointCount,
  //   sizeof(int));
  //
  //   for(int i = 0; i < context->activeJointCount; i++) {
  //     if(context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
  //       int parent = context->pcl[i];
  //       while(true) {
  //         if(parent == -1) {
  //           break;
  //         }
  //         if(context->jointPointers[i]->jointType != S2_JOINT_RIGID) {
  //           break;
  //         }
  //         parent = context->pcl[parent];
  //       }
  //       // printf("i: %d\n", i);
  //       // printf("parent: %d\n", parent);
  //       // fprintf(debug, "parent: %d\n", parent);
  //       if(parent == -1) {
  //         context->pclCompressed[context->jointLookup[i]] = -1;
  //       } else {
  //         context->pclCompressed[context->jointLookup[i]] =
  //           context->jointLookup[parent];
  //       }
  //     }
  //   }
  //
  //   /* create parent child list for dq array */
  //   if(context->pcldq) {
  //     free(context->pcldq);
  //   }
  //   context->pcldq = calloc(context->dqCount, sizeof(int));
  //
  //   for(int i = 0; i < context->activeJointCount; i++) {
  //     int parentIndex = context->pcl[i];
  //     int variableIndex = context->dqLookup[i];
  //     if(variableIndex != -1) {
  //       if(parentIndex != -1) {
  //         int jointDofOffset = fmax(context->jointDOF[parentIndex] - 1, 0);
  //         context->pcldq[variableIndex] =
  //           context->dqLookup[parentIndex] + jointDofOffset;
  //       } else {
  //         context->pcldq[variableIndex] = -1;
  //       }
  //     }
  //     int jointDOF = context->jointDOF[i];
  //     if(jointDOF > 1) {
  //       for(int j = 1; j < jointDOF; j++) {
  //         context->pcldq[variableIndex + j] = variableIndex + j - 1;
  //       }
  //     }
  //   }
  //
  //   /* update has mass dq to set true for any variable that has mass
  //   projected onto it */
  //   for(int k = context->dqCount - 1; k > -1; k--) {
  //     int j = context->pcldq[k];
  //     if(context->hasMassdq[k] && j > -1) {
  //       context->hasMassdq[j] = true;
  //     }
  //   }
  //
  //   /* set isRotation */
  //   if(context->isRotation) {
  //     free(context->isRotation);
  //   }
  //   context->isRotation = calloc(context->dqCount, sizeof(bool));
  //   for(int i = 0; i < context->activeJointCount; i++) {
  //     int baseIndex = context->dqLookup[i];
  //     switch(context->jointPointers[i]->jointType) {
  //       case S2_JOINT_TX:
  //       case S2_JOINT_TY:
  //       case S2_JOINT_TZ:
  //       case S2_JOINT_RIGID:
  //       case S2_JOINT_PRIMITIVE:
  //       case S2_JOINT_INVALID:
  //         break;
  //       case S2_JOINT_RX:
  //       case S2_JOINT_RY:
  //       case S2_JOINT_RZ: {
  //         context->isRotation[baseIndex] = true;
  //         break;
  //       }
  //       case S2_JOINT_SPHERICAL: {
  //         for(int j = 0; j < 3; j++) {
  //           context->isRotation[baseIndex + j] = true;
  //         }
  //         break;
  //       }
  //       case S2_JOINT_FLOAT: {
  //         for(int j = 3; j < 6; j++) {
  //           context->isRotation[baseIndex + j] = true;
  //         }
  //         break;
  //       }
  //     }
  //   }
  //
  //   for(int i = 0; i < context->activeJointCount; i++) {
  //     int compressedJointIndex = context->jointLookup[i];
  //     if(compressedJointIndex > -1) {
  //       context->jointDOFCompressed[compressedJointIndex] =
  //       context->jointDOF[i];
  //     }
  //   }
  //
  // #if _SHIFT_VERBOSE
  //   // fprintf(debug, "levels: ");
  //   // s2_arrayi_print(debug, context->levels, context->activeJointCount);
  //   fprintf(debug, "jointDOF: ");
  //   s2_arrayi_print(debug, context->activeJointCount, context->jointDOF);
  //   fprintf(debug, "jointDOFCompressed: ");
  //   s2_arrayi_print(debug, context->nonRigidJointCount,
  //                   context->jointDOFCompressed);
  //   fprintf(debug, "jointLookup: ");
  //   s2_arrayi_print(debug, context->activeJointCount, context->jointLookup);
  //   fprintf(debug, "dqLookup: ");
  //   s2_arrayi_print(debug, context->activeJointCount, context->dqLookup);
  //   fprintf(debug, "pcl: ");
  //   s2_arrayi_print(debug, context->activeJointCount, context->pcl);
  //   fprintf(debug, "dqLookupCompresed: ");
  //   s2_arrayi_print(debug, context->nonRigidJointCount,
  //                   context->dqLookupCompressed);
  //   fprintf(debug, "pclCompressed: ");
  //   s2_arrayi_print(debug, context->nonRigidJointCount,
  //   context->pclCompressed); fprintf(debug, "pcldq: ");
  //   s2_arrayi_print(debug, context->dqCount, context->pcldq);
  //   fprintf(debug, "hasMass: ");
  //   for(int i = 0; i < context->nonRigidJointCount; i++) {
  //     printf("%d ", context->hasMass[i] ? 1 : 0);
  //   }
  //   printf("\n");
  //   fprintf(debug, "hasMassdq: ");
  //   for(int i = 0; i < context->dqCount; i++) {
  //     printf("%d ", context->hasMassdq[i] ? 1 : 0);
  //   }
  //   printf("\n");
  //   printf("isRotation: ");
  //   for(int i = 0; i < context->dqCount; i++) {
  //     printf("%d ", context->isRotation[i] ? 1 : 0);
  //   }
  //   printf("\n");
  // #endif
}

void Shift::Jacobian::compressLoopMembers() {}

void Shift::Jacobian::setPO1() {}

void Shift::Jacobian::translatePermutations() {}

void Shift::Jacobian::setBooleanPO2() {}

void Shift::Jacobian::evaluateSparsity() {}

void Shift::Jacobian::setPO2() {}

void Shift::Jacobian::factorPO2() {}

void Shift::Jacobian::setBMatrixSparse() {}

void Shift::Jacobian::setConstraintPointers() {}

void Shift::Jacobian::setSubstructurePointers() {}
