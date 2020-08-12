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

#ifndef FAST9COLORKERNELLIST_H
#define FAST9COLORKERNELLIST_H

#include "fast9_acf.h"
#include "gauss_3x3_acf.h"
#include "nms_acf.h"
#include "mark_color_channel_acf.h"
#include "rgb_to_grayscale_acf.h"
#include "saturate_nonzero_acf.h"
#include "dilate_diamond_acf.h"

KERNEL_INFO kernelInfoConcat(RGB_TO_GRAY_K)
(
  RGB_TO_GRAY_KN,
  2,
  __port(__index(0),
         __identifier(RGB_TO_GRAY_KN_IN),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(3, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(RGB_TO_GRAY_KN_OUT),
         __attributes(ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO kernelInfoConcat(FAST9_K)
(
  FAST9_KN,
  3,
  __port(__index(0),
         __identifier(FAST9_KN_IN_IMG),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(3,3,3,3),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(FAST9_KN_OUT),
         __attributes(ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(2),
         __identifier(FAST9_KN_IN_THR),
         __attributes(ATTR_IN|ATTR_DMEM|ATTR_STATIC|ATTR_FIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO kernelInfoConcat(MARK_COL_K)
(
  MARK_COL_KN,
  4,
  __port(__index(0),
         __identifier(MARK_COL_KN_IN),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(3, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(MARK_COL_KN_MARKER),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(2),
         __identifier(MARK_COL_KN_CHN_IDX),
         __attributes(ATTR_IN|ATTR_DMEM|ATTR_STATIC|ATTR_FIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(3),
         __identifier(MARK_COL_KN_OUT),
         __attributes(ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(3, 1),
         __ek_size(1, 1))
);

KERNEL_INFO kernelInfoConcat(GAUSS_3x3_K)
(
  GAUSS_3x3_KN,
  2,
  __port(__index(0),
         __identifier(GAUSS_3x3_KN_IN),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(1,1,1,1),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(GAUSS_3x3_KN_OUT),
         __attributes(ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO kernelInfoConcat(NMS_K)
(
  NMS_KN,
  2,
  __port(__index(0),
         __identifier(NMS16_KN_IN),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(1,1,1,1),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(NMS16_KN_OUT),
         __attributes(ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO kernelInfoConcat(DILATE_DIAMOND_K)
(
  DILATE_DIAMOND_KN,
  2,
  __port(__index(0),
         __identifier(DILATE_DIAMOND_KN_IN),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(2,2,2,2),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(DILATE_DIAMOND_KN_OUT),
         __attributes(ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

KERNEL_INFO kernelInfoConcat(SAT_NONZERO_K)
(
  SAT_NONZERO_KN,
  2,
  __port(__index(0),
         __identifier(SAT_NONZERO_IN),
         __attributes(ATTR_IN|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1)),
  __port(__index(1),
         __identifier(SAT_NONZERO_OUT),
         __attributes(ATTR_OUT|ATTR_CMEM|ATTR_NONSTATIC|ATTR_NONFIXED|ATTR_REG),
         __spatial_dep(0,0,0,0),
         __e0_data_type(d08u),
         __e0_size(1, 1),
         __ek_size(1, 1))
);

#endif /* FAST9COLORKERNELLIST_H */
