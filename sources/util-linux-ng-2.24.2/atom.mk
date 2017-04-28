
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := util-linux-ng
LOCAL_DESCRIPTION := Miscellaneous utility programs
LOCAL_CATEGORY_PATH := system

LOCAL_AUTOTOOLS_VERSION := 2.24.2
LOCAL_AUTOTOOLS_ARCHIVE := util-linux-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := util-linux-$(LOCAL_AUTOTOOLS_VERSION)

# Update some licenses
define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_BSD.libuuid \
		$(PRIVATE_SRC_DIR)/libuuid/.MODULE_LICENSE_BSD
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.libblkid \
		$(PRIVATE_SRC_DIR)/libblkid/.MODULE_LICENSE_LGPL2
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.libmount \
		$(PRIVATE_SRC_DIR)/libmount/.MODULE_LICENSE_LGPL2
	$(Q) cp -af $(PRIVATE_PATH)/.MODULE_LICENSE_LGPL2.randutils.c \
		$(PRIVATE_SRC_DIR)/lib/.MODULE_LICENSE_LGPL2.randutils.c
endef

#list generated with ./configure --help | grep "do not build" | awk '{print "\t"$1" \\" }'
LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
        --disable-mount \
        --disable-losetup \
        --disable-cytune \
        --disable-fsck \
        --disable-partx \
        --disable-uuidd \
        --disable-mountpoint \
        --disable-fallocate \
        --disable-unshare \
        --disable-nsenter \
        --disable-setpriv \
        --disable-eject \
        --disable-agetty \
        --disable-cramfs \
        --disable-bfs \
        --disable-fdformat \
        --disable-hwclock \
        --disable-wdctl \
        --disable-switch_root \
        --disable-pivot_root \
        --disable-kill \
        --disable-last \
        --disable-utmpdump \
        --disable-mesg \
        --disable-raw \
        --disable-rename \
        --disable-login \
        --disable-nologin \
        --disable-sulogin \
        --disable-su \
        --disable-runuser \
        --disable-ul \
        --disable-more \
        --disable-pg \
        --disable-setterm \
        --disable-schedutils \
        --disable-wall \
        --disable-rpath \
        --without-ncurses \
        --without-udev

#        --disable-libuuid \
#        --disable-libblkid \
#        --disable-libmount \

ifeq ("$(TARGET_OS_FLAVOUR)","native")
LOCAL_AUTOTOOLS_CONFIGURE_ARGS += \
	--prefix=$(TARGET_OUT_STAGING) \
	--includedir=$(TARGET_OUT_STAGING)/usr/include \
	--datarootdir=$(TARGET_OUT_STAGING)/usr/share
else
LOCAL_AUTOTOOLS_CONFIGURE_ARGS += \
	--prefix=/ \
	--includedir=/usr/include \
	--datarootdir=/usr/share
endif

# Only install a subset
# use a local DESTDIR to not overide busybox binary
define LOCAL_AUTOTOOLS_CMD_INSTALL
	$(Q) $(AUTOTOOLS_MAKE_ENV) $(MAKE) -C $(PRIVATE_SRC_DIR) \
		$(AUTOTOOLS_MAKE_ARGS) DESTDIR=$(PRIVATE_SRC_DIR)/install install
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/sbin
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/lib
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/lib/pkgconfig
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/blkid
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/libmount
	$(Q) install -p -m755 -d $(TARGET_OUT_STAGING)/usr/include/uuid
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/lib/libblkid.so.1.1.0 $(TARGET_OUT_STAGING)/lib/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/lib/libmount.so.1.1.0 $(TARGET_OUT_STAGING)/lib/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/lib/libuuid.so.1.3.0 $(TARGET_OUT_STAGING)/lib/
	$(Q) ln -sf libblkid.so.1.1.0 $(TARGET_OUT_STAGING)/lib/libblkid.so.1
	$(Q) ln -sf libblkid.so.1.1.0 $(TARGET_OUT_STAGING)/lib/libblkid.so
	$(Q) ln -sf libmount.so.1.1.0 $(TARGET_OUT_STAGING)/lib/libmount.so.1
	$(Q) ln -sf libmount.so.1.1.0 $(TARGET_OUT_STAGING)/lib/libmount.so
	$(Q) ln -sf libuuid.so.1.3.0 $(TARGET_OUT_STAGING)/lib/libuuid.so.1
	$(Q) ln -sf libuuid.so.1.3.0 $(TARGET_OUT_STAGING)/lib/libuuid.so
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/lib/pkgconfig/blkid.pc $(TARGET_OUT_STAGING)/lib/pkgconfig/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/lib/pkgconfig/uuid.pc $(TARGET_OUT_STAGING)/lib/pkgconfig/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/lib/pkgconfig/mount.pc $(TARGET_OUT_STAGING)/lib/pkgconfig/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/lib/pkgconfig/mount.pc $(TARGET_OUT_STAGING)/lib/pkgconfig/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/blkid/blkid.h $(TARGET_OUT_STAGING)/usr/include/blkid/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/libmount/libmount.h $(TARGET_OUT_STAGING)/usr/include/libmount/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/usr/include/uuid/uuid.h $(TARGET_OUT_STAGING)/usr/include/uuid/
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/bin/dmesg $(TARGET_OUT_STAGING)/sbin/dmesg-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/blkid $(TARGET_OUT_STAGING)/sbin/blkid-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/fstrim $(TARGET_OUT_STAGING)/sbin/fstrim-ng
	$(Q) install -p -m755 $(PRIVATE_SRC_DIR)/install/sbin/fdisk $(TARGET_OUT_STAGING)/sbin/fdisk-ng
endef

# Clean what we manually installed
# Libraries are cleaned because the uninstall is done on staging...
define LOCAL_AUTOTOOLS_CMD_POST_CLEAN
	$(Q) rm -f $(TARGET_OUT_STAGING)/sbin/blkid-ng
	$(Q) rm -f $(TARGET_OUT_STAGING)/sbin/dmesg-ng
	$(Q) rm -f $(TARGET_OUT_STAGING)/sbin/fstrim-ng
endef

LOCAL_CREATE_LINKS := etc/mtab:/proc/mounts

include $(BUILD_AUTOTOOLS)

