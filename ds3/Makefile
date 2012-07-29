#
# Copyright (C) 2006-2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=ds3
PKG_VERSION:=1.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/ds3
  SECTION:=utils
  CATEGORY:=Utilities
  DEPENDS:=+libusb
  TITLE:=Dualshock PS3 Driver
  URL:=https://github.com/genko/ds3
endef
	
define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

EXTRA_CFLAGS += $(TARGET_CPPFLAGS)
EXTRA_LDFLAGS += $(TARGET_LDFLAGS)

define Build/Compile
	$(call Build/Compile/Default, \
		CCOPTS="$(TARGET_CFLAGS)" \
		INCLUDE="$(EXTRA_CFLAGS)" \
		LDFLAGS="$(EXTRA_LDFLAGS)" \
	)
endef

define Package/ds3/install	
	$(INSTALL_DIR) $(1)/usr/bin
	$(CP) $(PKG_BUILD_DIR)/$(PKG_NAME) $(1)/usr/bin/
endef

$(eval $(call BuildPackage,ds3))
