# OpenSim-based full-body multibody model built in Artisynth
This repository holds a fork of the original [artisynth_models](https://github.com/artisynth/artisynth_models) repository by John Lloyd containing a full-body model based on the gait2392.osim model in OpenSim. The model was built using the freely available 3D modelling platform [ArtiSynth](https://www.artisynth.org/Main/HomePage). The model is intended for combined multi-body and finite element simulations and contains contact and constraint definitions.

If you use the model or parts of it in your research, please cite the following reference:
> Denk A., Kowalczyk W., 2024, OpenSim-based full-body multibody model built in Artisynth, GitHub repository (https://github.com/AlexDenk/artisynth_models).

![General model overview](Gait2392_Demo.gif)

The image shows the generated model in Artisynth under parametric control. Inverse controlling is also available for the model but is still under development.

![Inverse Simulation overview](Gait2392_Demo.png)

## What this repository includes
Since being a fork, this repository contains the original artisynth_models repository, alongside:
* The [gait2392 model](src/artisynth/models/diss) and all necessary geometries including the upper extremities
* The packages to read and process .mot files including an updated TRC-Reader
* Exemplary motion-capturing data from tutorial 1 of the [gait2392](https://simtk.org/frs/?group_id=91) model (Delp, S.L., Anderson, F.C., Arnold, A.S., Loan, P., Habib, A., John, C.T., Guendelman, E., Thelen, D.G. OpenSim: Open-source software to create and analyze dynamic simulations of movement. IEEE Transactions on Biomedical Engineering, 54(11), pp 1940-1950. (2007)).

## License
This source code is licensed under the license found in the [LICENSE file](LICENSE) in the root directory of this source tree.
