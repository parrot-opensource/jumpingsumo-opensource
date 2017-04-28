
LOCAL_PATH := $(call my-dir)

# Linux only without android
ifeq ("$(TARGET_OS)","linux")
ifneq ("$(TARGET_OS_FLAVOUR)","android")

###############################################################################
# alsa-libs
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := alsa-lib
LOCAL_DESCRIPTION := Alsa library
LOCAL_CATEGORY_PATH := audio

LOCAL_EXPORT_LDLIBS := -lasound

LOCAL_AUTOTOOLS_VERSION := 1.0.26
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	alsa-lib-dis_wordexp.patch 	\
	alsa-lib-dlsym.patch

LOCAL_AUTOTOOLS_CONFIGURE_ENV :=

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--disable-aload \
	--disable-alisp \
	--disable-python \
	--with-debug \
	--enable-symbolic-functions

# Put police files (can't use automatic copy from LOCAL_PATH because we have 2
# different modules and licenses in 2 archives here)
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) touch $(PRIVATE_SRC_DIR)/.MODULE_NAME_alsa-lib
	$(Q) touch $(PRIVATE_SRC_DIR)/.MODULE_LICENSE_LGPL
endef

LOCAL_CLEAN_FILES := \
	$(TARGET_OUT_STAGING)/usr/include/sys/asoundlib.h

include $(BUILD_AUTOTOOLS)

###############################################################################
# alsa-libs-static
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := alsa-lib-static
LOCAL_DESCRIPTION := Alsa library static
LOCAL_CATEGORY_PATH := audio

LOCAL_EXPORT_LDLIBS := -lasound

LOCAL_AUTOTOOLS_VERSION := 1.0.26
LOCAL_AUTOTOOLS_ARCHIVE := alsa-lib-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := alsa-lib-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	alsa-lib-dis_wordexp.patch 	\
	alsa-lib-dlsym.patch

LOCAL_AUTOTOOLS_CONFIGURE_ENV :=

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--disable-aload \
	--disable-alisp \
	--disable-python \
	--with-debug \
	--enable-symbolic-functions \
	--enable-shared=no --enable-static \
	--enable-static-nss --with-libdl=no \
	--with-pcm-plugins="plug rate route" \
	--with-ctl-plugins=no \
	--with-pthread=no

# Put police files (can't use automatic copy from LOCAL_PATH because we have 2
# different modules and licenses in 2 archives here)
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) touch $(PRIVATE_SRC_DIR)/.MODULE_NAME_alsa-lib
	$(Q) touch $(PRIVATE_SRC_DIR)/.MODULE_LICENSE_LGPL
endef

LOCAL_CLEAN_FILES := \
	$(TARGET_OUT_STAGING)/usr/include/sys/asoundlib.h

include $(BUILD_AUTOTOOLS)

###############################################################################
# alsa-utils
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := alsa-utils
LOCAL_DESCRIPTION := Alsa utility
LOCAL_CATEGORY_PATH := audio/tools
LOCAL_LIBRARIES := alsa-lib

LOCAL_AUTOTOOLS_VERSION := 1.0.26
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_CONFIG_FILES := ConfigAlsaUtils.in
$(call load-config)

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--disable-nls \
	--disable-alsatest \
	--disable-xmlto

ifdef CONFIG_ALSAMIXER
	LOCAL_LIBRARIES += ncurses
	# Prevent using ncurses5-config found on host and creating cross-compilation mess
	LOCAL_AUTOTOOLS_CONFIGURE_ARGS += \
		  ac_cv_prog_ncurses5_config=dummy
else
	LOCAL_AUTOTOOLS_CONFIGURE_ARGS += --disable-alsamixer
endif

LOCAL_AUTOTOOLS_PATCHES := \
	alsa-utils-alsactl_directory.patch

# Put police files (can't use automatic copy from LOCAL_PATH because we have 2
# different modules and licenses in 2 archives here)
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) touch $(PRIVATE_SRC_DIR)/.MODULE_NAME_alsa-utils
	$(Q) touch $(PRIVATE_SRC_DIR)/.MODULE_LICENSE_GPL
endef

LOCAL_CLEAN_FILES := \
	$(TARGET_OUT_STAGING)/usr/bin/arecord

LOCAL_COPY_FILES += \
        20-alsactl.rc:etc/boxinit.d/

include $(BUILD_AUTOTOOLS)

endif
endif

