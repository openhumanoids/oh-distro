#include "RendererGroupUtil.hpp"

#include <string>
#include <vector>
#include <cstring>
#include <gtk/gtk.h>
#include <bot_vis/viewer.h>
#include <bot_param/param_client.h>

struct RendererGroupUtil::ViewerBundle {
  BotViewer* mViewer;
  BotParam* mParam;

  std::vector<std::string>
  getEnabled(const std::string& iLabel) {
    std::vector<std::string> enabled;
    std::string keyBase = "viewer.renderer_groups";
    char** groups = bot_param_get_subkeys(mParam, keyBase.c_str());
    if (groups != NULL) {
      for (int i = 0; (groups[i] != NULL); ++i) {
        std::string key = keyBase + "." + groups[i] + ".name";
        char* value = NULL;
        if (0 == bot_param_get_str(mParam, key.c_str(), &value)) {
          if (iLabel == value) {
            key = keyBase + "." + groups[i] + ".enabled";
            char** names = bot_param_get_str_array_alloc(mParam, key.c_str());
            if (names != NULL) {
              for (int j = 0; (names[j] != NULL); ++j) {
                enabled.push_back(names[j]);
              }
              g_strfreev(names);
            }
          }
          free(value);
        }
      }
      g_strfreev(groups);
    }
    return enabled;
  }

  static void onMenuChange(GtkMenuItem* iItem, void* iUser) {
    ViewerBundle* bundle = (ViewerBundle*)iUser;
    BotViewer* viewer = bundle->mViewer;
    BotParam* param = bundle->mParam;

    const char* label = gtk_menu_item_get_label(iItem);
    if (!strcmp(label, "No Group")) return;

    std::vector<std::string> rendererNames = bundle->getEnabled(label);

    // enable and disable relevant renderers
    for (unsigned int i = 0; i < viewer->renderers->len; ++i) {
      BotRenderer* renderer =
        (BotRenderer*)g_ptr_array_index(viewer->renderers, i);

      bool found = false;
      for (size_t j = 0; j < rendererNames.size(); ++j) {
        if (rendererNames[j] == renderer->name) {
          found = true;
          break;
        }
      }

      renderer->enabled = found;
      if (renderer->widget) {
        GtkWidget* frame =
          (GtkWidget*)g_object_get_data(G_OBJECT(renderer->widget), 
                                        "BotViewer:frame");
        if (frame) {
          if (found) gtk_widget_show(frame);
          else gtk_widget_hide(frame);
        }
      }
      GtkCheckMenuItem* cmi = GTK_CHECK_MENU_ITEM(renderer->cmi);
      if (cmi) {
        gtk_check_menu_item_set_active (cmi, found);
      }
    }

    // enable and disable relevant inputs
    for (unsigned int i = 0; i < viewer->event_handlers_sorted->len; ++i) {
      BotEventHandler* handler =
        (BotEventHandler*)g_ptr_array_index(viewer->event_handlers_sorted, i);

      bool found = false;
      for (size_t j = 0; j < rendererNames.size(); ++j) {
        if (rendererNames[j] == handler->name) {
          found = true;
          break;
        }
      }

      handler->enabled = found;
      gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(handler->cmi), found);
    }
  }
};

RendererGroupUtil::
RendererGroupUtil(BotViewer* iViewer, BotParam* iParam) {
  mBundle.reset(new ViewerBundle());
  mBundle->mViewer = iViewer;
  mBundle->mParam = iParam;
}

void RendererGroupUtil::
setup() {
  BotViewer* viewer = mBundle->mViewer;
  BotParam* param = mBundle->mParam;

  // set up top level menu
  GtkWidget* groupMenuItem = gtk_menu_item_new_with_mnemonic("_Groups");
  gtk_menu_bar_append(GTK_MENU_BAR(viewer->menu_bar), groupMenuItem);
  GtkWidget* groupMenu = gtk_menu_new();
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(groupMenuItem), groupMenu);

  // add tearoff
  GtkWidget* tearoff = gtk_tearoff_menu_item_new();
  gtk_menu_append(groupMenu, tearoff);
  gtk_widget_show(tearoff);

  // get all group names and info
  std::vector<std::string> groupNames;
  groupNames.push_back("No Group");
  std::string keyBase = "viewer.renderer_groups";
  char** groups = bot_param_get_subkeys(param, keyBase.c_str());
  if (groups != NULL) {
    for (int i = 0; (groups[i] != NULL); ++i) {
      std::string key = keyBase + "." + groups[i] + ".name";
      char* value = NULL;
      if (0 == bot_param_get_str(param, key.c_str(), &value)) {
        groupNames.push_back(value);
        free(value);
      }
    }
    g_strfreev(groups);
  }

  // set up menu items
  GSList *groupList = NULL;
  for (size_t i = 0; i < groupNames.size(); ++i) {
    GtkWidget *groupItem =
      gtk_radio_menu_item_new_with_label(groupList, groupNames[i].c_str());
    groupList = gtk_radio_menu_item_get_group(GTK_RADIO_MENU_ITEM(groupItem));
    gtk_menu_append(GTK_MENU(groupMenu), groupItem);
    g_signal_connect(G_OBJECT(groupItem), "activate",
                     G_CALLBACK(ViewerBundle::onMenuChange), mBundle.get());
  }

  // show the menu
  gtk_widget_show_all(groupMenuItem);
}
