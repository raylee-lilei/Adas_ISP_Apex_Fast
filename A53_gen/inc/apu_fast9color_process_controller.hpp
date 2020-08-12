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

#ifndef APUFAST9COLORPROCESSCONTROLLER_HPP
#define APUFAST9COLORPROCESSCONTROLLER_HPP

#include "fast9color_graph_names.h"
#include <umat.hpp>

#ifdef APEX2_EMULATE
#include "fast9color_process_type.h"
#else
#include str_header(FAST9COLOR_PI,hpp)
#include <icp_data.h>
#include <apex.h>
using namespace icp;
#endif

namespace apugen {

class APU_Fast9Color_Process_Controller {
private:
  FAST9COLOR_PI * pProc;
public:
  APU_Fast9Color_Process_Controller(int32_t apex_id = 0);
  ~APU_Fast9Color_Process_Controller();
  int32_t Init();
  int32_t Connect(vsdk::UMat& dataInput, vsdk::UMat& dataThreshold, vsdk::UMat& dataMarkColorChannel, vsdk::UMat& dataOutput);
  int32_t Start();
  int32_t Wait();
};

}

#endif /* APUFAST9COLORPROCESSCONTROLLER_HPP */

