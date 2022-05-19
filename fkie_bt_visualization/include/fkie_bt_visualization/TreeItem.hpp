// Copyright 2022 Fraunhofer FKIE - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TREE_ITEM_H
#define TREE_ITEM_H

#include <iostream>
#include <memory>
#include <vector>

#include <QAbstractItemModel>
#include <QDomDocument>
#include <QFile>
#include <QModelIndex>
#include <QStandardItemModel>
#include <QTreeView>
#include <QVariant>

class TreeItem
{
public:
  std::string node_type;
  std::string node_type_view;
  std::string node_ID;
  std::string node_name;
  std::string node_name_view;

  TreeItem(std::string node_type, std::string node_ID, std::string node_name)
    : node_type(node_type), node_type_view(node_type), node_ID(node_ID), node_name(node_name), node_name_view(node_name)
  {
    clearName();
  }

  TreeItem(QString node_type, QString node_ID, QString node_name)
    : TreeItem(node_type.toStdString(), node_ID.toStdString(), node_name.toStdString())
  {
    clearName();
  }

  TreeItem(QDomNode node)
    : TreeItem(node.toElement().tagName(), node.toElement().attribute(QString("ID")),
               node.toElement().attribute(QString("name")))
  {
    clearName();
  }

  bool isValid() const
  {
    return !node_type.empty() || !node_ID.empty() || !node_name.empty();
  }

  void clearName()
  {
    if (node_type == "Action")
    {
      node_type_view = "|";
      node_name_view = "[A] " + node_name;
    }

    if (node_type == "Condition")
    {
      node_type_view = "|";
      node_name_view = "[C] " + node_name;
    }
  }

  /**
   * @brief Computes an "unique" string from properties. It might not be unique if the XML tree is not proper configured
   */
  std::string getStrID() const
  {
    // return node_type + "_" + node_name + "_" + node_ID;
    return node_name;  // so far, only name is consistent
  }

  QString toQString() const
  {
    return QString("Node:%0, ID:%1, Name:%2")
        .arg(QString(node_type.c_str()))
        .arg(QString(node_ID.c_str()))
        .arg(QString(node_name.c_str()));
  }

  std::string toString() const
  {
    std::string r;
    r += " Type: " + node_type;
    r += " Name: " + node_name;
    r += " ID: " + node_ID;
    return r;
  }
};

#endif  // TREE_ITEM_H
