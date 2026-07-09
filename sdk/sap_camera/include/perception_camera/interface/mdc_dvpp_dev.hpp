/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Qin Chen <qinchen@senseauto.com>
 */
#pragma once
#include "core/core.h"
// #include "dvpp/HiDvppVpc.h"
// #include "dvpp/HiDvppSys.h"
#include "dvpp/hi_dvpp.h"
#include "ad_log/ad_log.hpp"

#define HI_CHECK(Func)                                             \
    do {                                                           \
        auto status = Func;                                        \
        if (status != 0) {                                         \
            AP_LERROR(MPIChannel)                                  \
                << #Func << " error! code:" << std::hex << status; \
            return false;                                          \
        }                                                          \
    } while (0);

namespace sensetime {
namespace interface {

struct MDCDVPPDesc {
    hi_vpc_chn_attr mpi_attr_;
    hi_vpc_chn mpi_channel_;
};

class MPIChannel {
 private:
    MDCDVPPDesc mdc_dvpp_desc_;

 public:
    static MPIChannel *GetInstance() {
        static MPIChannel mpi_channel;
        return &mpi_channel;
    }

    bool Deinit() {
        HI_CHECK(hi_mpi_vpc_destroy_chn(mdc_dvpp_desc_.mpi_channel_));
        HI_CHECK(hi_mpi_sys_exit());
        return true;
    }

    bool Init(hi_vpc_chn mpi_channel) {
        //  MPI interface initialize
        HI_CHECK(hi_mpi_sys_init());

        mdc_dvpp_desc_.mpi_channel_ = mpi_channel;
        mdc_dvpp_desc_.mpi_attr_.pic_width = 3840;
        mdc_dvpp_desc_.mpi_attr_.pic_height = 2160;
        HI_CHECK(hi_mpi_vpc_create_chn(mdc_dvpp_desc_.mpi_channel_,
                                       &mdc_dvpp_desc_.mpi_attr_));
        AP_LDEBUG(MPIChannel) << " Init mdc_dvpp_desc_.mpi_channel_="
                              << mdc_dvpp_desc_.mpi_channel_;
        return true;
    }

    MDCDVPPDesc GetDesc() { return mdc_dvpp_desc_; }
};

}  // namespace interface
}  // namespace sensetime
