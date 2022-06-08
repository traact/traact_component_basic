/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/
#include "FeaturelessOutsideInModule.h"
namespace traact::component {
class FeaturelessOutsideInCamera : public FeaturelessOutsideInComponentInput {
 public:

    explicit FeaturelessOutsideInCamera(const std::string &name) : FeaturelessOutsideInComponentInput(name) {

    };

 protected:

 RTTR_ENABLE(FeaturelessOutsideInComponent)
};
}