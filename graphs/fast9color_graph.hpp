
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

/*!*********************************************************************************
*  @file    fast9color_graph.hpp
*  @brief   ACF graph
***********************************************************************************/

#ifndef FAST9COLORGRAPH_HPP
#define FAST9COLORGRAPH_HPP

#include <acf_graph.hpp>

#ifdef APEX2_EMULATE
#include "acf_lib.hpp"
using namespace APEX2;
#endif

#include "fast9color_graph_names.h"

#include "fast9_acf.h"
#include "gauss_3x3_acf.h"
#include "nms_acf.h"
#include "mark_color_channel_acf.h"
#include "rgb_to_grayscale_acf.h"
#include "saturate_nonzero_acf.h"
#include "dilate_diamond_acf.h"

class Fast9colorGraph: public ACF_Graph
{
public:
  Fast9colorGraph() : ACF_Graph()
  {
    XREGISTER_ACF_KERNEL(RGB_TO_GRAY_K);
    XREGISTER_ACF_KERNEL(FAST9_K);
    XREGISTER_ACF_KERNEL(MARK_COL_K);
    XREGISTER_ACF_KERNEL(GAUSS_3x3_K);
    XREGISTER_ACF_KERNEL(NMS_K);
    XREGISTER_ACF_KERNEL(DILATE_DIAMOND_K);
    XREGISTER_ACF_KERNEL(SAT_NONZERO_K);
  }

  void Create()
  {
    //set identifier for graph
    SetIdentifier("Fast9colorGraph");

    //add kernels
    const std::string rgb_to_grayscale_0("rgb_to_grayscale_0");
    AddKernel(rgb_to_grayscale_0, RGB_TO_GRAY_KN);
    const std::string fast9_0("fast9_0");
    AddKernel(fast9_0, FAST9_KN);
    const std::string mark_color_channel_0("mark_color_channel_0");
    AddKernel(mark_color_channel_0, MARK_COL_KN);
    const std::string gauss_3x3_0("gauss_3x3_0");
    AddKernel(gauss_3x3_0, GAUSS_3x3_KN);
    const std::string nms_0("nms_0");
    AddKernel(nms_0, NMS_KN);
    const std::string dilate_diamond_0("dilate_diamond_0");
    AddKernel(dilate_diamond_0, DILATE_DIAMOND_KN);
    const std::string dilate_diamond_1("dilate_diamond_1");
    AddKernel(dilate_diamond_1, DILATE_DIAMOND_KN);
    const std::string saturate_nonzero_0("saturate_nonzero_0");
    AddKernel(saturate_nonzero_0, SAT_NONZERO_KN);

    //add graph ports
    AddInputPort(GR_INPUT_IN);
    AddInputPort(GR_THRESHOLD_IN);
    AddInputPort(GR_MARKCOLORCHANNEL_IN);
    AddOutputPort(GR_OUTPUT_OUT);

    //specify connections
    Connect(GraphPort(GR_INPUT_IN), KernelPort(rgb_to_grayscale_0, RGB_TO_GRAY_KN_IN));
    Connect(GraphPort(GR_INPUT_IN), KernelPort(mark_color_channel_0, MARK_COL_KN_IN));
    Connect(GraphPort(GR_THRESHOLD_IN), KernelPort(fast9_0, FAST9_KN_IN_THR));
    Connect(GraphPort(GR_MARKCOLORCHANNEL_IN), KernelPort(mark_color_channel_0, MARK_COL_KN_CHN_IDX));
    Connect(KernelPort(rgb_to_grayscale_0, RGB_TO_GRAY_KN_OUT), KernelPort(gauss_3x3_0, GAUSS_3x3_KN_IN));
    Connect(KernelPort(fast9_0, FAST9_KN_OUT), KernelPort(nms_0, NMS16_KN_IN));
    Connect(KernelPort(gauss_3x3_0, GAUSS_3x3_KN_OUT), KernelPort(fast9_0, FAST9_KN_IN_IMG));
    Connect(KernelPort(nms_0, NMS16_KN_OUT), KernelPort(dilate_diamond_0, DILATE_DIAMOND_KN_IN));
    Connect(KernelPort(dilate_diamond_0, DILATE_DIAMOND_KN_OUT), KernelPort(dilate_diamond_1, DILATE_DIAMOND_KN_IN));
    Connect(KernelPort(dilate_diamond_1, DILATE_DIAMOND_KN_OUT), KernelPort(saturate_nonzero_0, SAT_NONZERO_IN));
    Connect(KernelPort(saturate_nonzero_0, SAT_NONZERO_OUT), KernelPort(mark_color_channel_0, MARK_COL_KN_MARKER));
    Connect(KernelPort(mark_color_channel_0, MARK_COL_KN_OUT), GraphPort(GR_OUTPUT_OUT));
  }
};

#endif /* FAST9COLORGRAPH_HPP */
