/*****************************************************************************
 *
 * Copyright 2020 NXP
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
*****************************************************************************/

#include <opencv2/opencv.hpp>
#include <umat.hpp>
#include <iostream>
#include <ostream>

#include "../../A53_gen/inc/apu_fast9color_process_controller.hpp"

using namespace cv;
using namespace std;
using namespace apugen;

APU_Fast9Color_Process_Controller::APU_Fast9Color_Process_Controller(int32_t apex_id) {
#ifdef APEX2_EMULATE
  pProc = new FAST9COLOR_PI((APEX_ID)apex_id);
#else 
  pProc = new FAST9COLOR_PI(apex_id);
#endif
}

APU_Fast9Color_Process_Controller::~APU_Fast9Color_Process_Controller() {
  delete pProc;
}

int32_t APU_Fast9Color_Process_Controller::Init() {
  int32_t lRetVal = 0;
  lRetVal |= pProc->Initialize();

  return lRetVal;
}

int32_t APU_Fast9Color_Process_Controller::Connect(vsdk::UMat& dataInput, vsdk::UMat& dataThreshold, vsdk::UMat& dataMarkColorChannel, vsdk::UMat& dataOutput) {
  int32_t lRetVal = 0;
  lRetVal |= pProc->ConnectIO(GR_INPUT_IN, dataInput);
  lRetVal |= pProc->ConnectIO(GR_THRESHOLD_IN, dataThreshold);
  lRetVal |= pProc->ConnectIO(GR_MARKCOLORCHANNEL_IN, dataMarkColorChannel);
  lRetVal |= pProc->ConnectIO(GR_OUTPUT_OUT, dataOutput);


  return lRetVal;
}

int32_t APU_Fast9Color_Process_Controller::Start() {
  return pProc->Start();
}

int32_t APU_Fast9Color_Process_Controller::Wait() {
  return pProc->Wait();
}

