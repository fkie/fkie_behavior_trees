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

#ifndef TREE_MODEL_H
#define TREE_MODEL_H

#include <iostream>
#include <memory>
#include <vector>

#include <QAbstractItemModel>
#include <QDomDocument>
#include <QFile>
#include <QFontMetrics>
#include <QModelIndex>
#include <QStandardItemModel>
#include <QString>
#include <QTreeView>
#include <QVariant>

#include "TreeItem.hpp"

class TreeModel
{
protected:
  std::unique_ptr<QStandardItemModel> bt_model;
  std::unique_ptr<QStandardItemModel> bt_model_compact;
  std::map<std::string, int> map_str_node_to_id_row;
  int last_active_row = 0;
  int c_row = 0;

  std::vector<int> size_longest_text_per_column{ 0, 0, 0 };
  std::vector<int> size_column_compact{ 0 };

public:
  TreeModel()
  {
    bt_model = std::make_unique<QStandardItemModel>();
    bt_model_compact = std::make_unique<QStandardItemModel>();
  }

  void clearModels()
  {
    (*bt_model).clear();
    (*bt_model_compact).clear();
  }

  /**
   * @brief Check if current model is valid (has been updated)
   */
  bool isValid() const
  {
    return bt_model->rowCount() > 0 && bt_model->columnCount() > 0 && bt_model_compact->rowCount() > 0;
  }

  /**
   * @brief Return current bt model
   */
  QStandardItemModel* getModel()
  {
    return bt_model.get();
  }

  /**
   * @brief Return current compact bt model
   */
  QStandardItemModel* getCompactModel()
  {
    return bt_model_compact.get();
  }

  /**
   * @brief Return current bt model
   */
  std::vector<int> getColumnSize() const
  {
    return std::vector<int>{
      (int)(size_longest_text_per_column[0] * 1.2),
      (int)(size_longest_text_per_column[1] * 1.2),
      (int)(size_longest_text_per_column[2] * 1.2),
    };
  }

  /**
   * @brief Return current bt model
   */
  std::vector<int> getColumnSizeCompact() const
  {
    return std::vector<int>{ (int)(size_column_compact[0] * 1.2) };
  }

  /**
   * @brief Return the number of rows in current view
   */
  double getRowCount()
  {
    return bt_model->rowCount();
  }

  /**
   * @brief highlight the current leaf node in execution. It uses [ti.getStrID()] to find the row on the TreeView and
   * update color accordingly
   */
  void setActiveAction(TreeItem ti)
  {
    if (map_str_node_to_id_row.count(ti.getStrID()) > 0)
    {
      int row = map_str_node_to_id_row.at(ti.getStrID());
      QBrush active_node_brush(QColor(250, 237, 165));

      // comple model
      QStandardItem* item_old = bt_model->item(last_active_row, 1);
      if (item_old)
        item_old->setBackground(QBrush(Qt::white));

      QStandardItem* item_row = bt_model->item(row, 1);
      if (item_row)
        item_row->setBackground(active_node_brush);

      // compact model
      QStandardItem* item_old_compact = bt_model_compact->item(last_active_row, 0);
      if (item_old_compact)
        item_old_compact->setBackground(QBrush(Qt::white));

      QStandardItem* item_row_compact = bt_model_compact->item(row, 0);
      if (item_row_compact)
        item_row_compact->setBackground(active_node_brush);

      last_active_row = row;
    }
    else
    {
      // qDebug("Ignoring node: %s", ti.getStrID().c_str());
    }
  }

  /**
   * @brief Initializes a Behavior Tree model View from a file located in [file_path]
   */
  [[nodiscard]] bool createBehaviorTreeModelFromFile(const std::string file_path)
  {
    bool success = false;

    QFile file_xml(file_path.c_str());
    QDomDocument doc("bt");
    if (file_xml.open(QIODevice::ReadOnly))
    {
      if (doc.setContent(&file_xml))
      {
        success = createBehaviorTreeModel(doc);
        if (success)
          qDebug("createBehaviorTreeModelFromFile: Behavior Tree was succesfully loaded");
      }
      else
      {
        qDebug("createBehaviorTreeModelFromFile: Could not set XML content to [QDomDocument]");
      }

      file_xml.close();
    }
    else
    {
      qDebug("createBehaviorTreeModelFromFile: Could not open file: [%s]", file_path.c_str());
    }

    return success;
  }

  /**
   * @brief Initializes a Behavior Tree model View from text [xml_text]
   */
  [[nodiscard]] bool createBehaviorTreeModelFromText(const QString& xml_text)
  {
    bool success = false;

    QDomDocument doc("bt");
    if (doc.setContent(xml_text))
    {
      success = createBehaviorTreeModel(doc);
      if (success)
        qDebug("createBehaviorTreeModelFromFile: Behavior Tree was succesfully loaded");
    }
    else
    {
      qDebug("createBehaviorTreeModelFromText: Could not set content: [%s]", qPrintable(xml_text));
    }

    return success;
  }

  /**
   * @brief Initializes a Behavior Tree model using XML document [doc]
   */
  [[nodiscard]] bool createBehaviorTreeModel(QDomDocument doc)
  {
    clearModels();

    // initializes complete model
    bt_model->insertColumn(0);
    bt_model->insertColumn(1);
    bt_model->insertColumn(2);

    bt_model->setHorizontalHeaderItem(0, new QStandardItem("Type"));
    bt_model->setHorizontalHeaderItem(1, new QStandardItem("Name"));
    bt_model->setHorizontalHeaderItem(2, new QStandardItem("ID"));

    // initializes compact model
    bt_model_compact->insertColumn(0);
    bt_model_compact->setHorizontalHeaderItem(0, new QStandardItem("Node"));

    std::vector<QDomNode> trees = findBehaviorTrees(doc);

    qDebug("createBehaviorTreeModel: Number of trees: [%ld]", trees.size());
    QDomNode main_n;
    for (QDomNode t : trees)
    {
      main_n = t;
      if (t.hasAttributes())
        main_n = convertSubTree(main_n, 0);
    }
    return isValid();
  }

  /**
   * @brief Returns all available sub-trees
   */
  std::vector<QDomNode> findBehaviorTrees(const QDomDocument& dom)
  {
    std::vector<QDomNode> trees;
    QDomElement docElem = dom.documentElement();

    for (int i = 0; i < docElem.childNodes().length(); i++)
    {
      QDomNode n = docElem.childNodes().at(i);
      if (!n.isNull() && n.toElement().tagName().toStdString() == "BehaviorTree")
        trees.push_back(n);
    }

    return trees;
  }

  /**
   * @brief Recursively extract items from XML tree and add them to bt_model
   */
  QDomNode convertSubTree(const QDomNode& node, size_t level)
  {
    TreeItem ti(node);
    if (!ti.isValid())
      return node;

    QFont font("Courier");
    font.setPointSize(11);

    QList<QStandardItem*> q_node_info;
    std::string blanks(level * 2, ' ');
    std::string blanks_compact(level * 1, ' ');
    QStandardItem* i_type = new QStandardItem(QString((blanks + ti.node_type_view).c_str()));
    font.setWeight(QFont::Bold);
    i_type->setForeground(Qt::darkBlue);
    i_type->setFont(font);
    int i_type_width = QFontMetrics(font).width(i_type->text());

    QStandardItem* i_name = new QStandardItem(QString(ti.node_name_view.c_str()));
    if (node.hasChildNodes())
    {
      font.setWeight(QFont::Bold);
      font.setPointSize(11);
    }
    else
    {
      font.setWeight(QFont::Normal);
      font.setPointSize(10);
    }

    i_name->setForeground(Qt::black);
    i_name->setFont(font);
    int i_name_width = QFontMetrics(font).width(i_name->text());

    QStandardItem* i_id = new QStandardItem(QString(ti.node_ID.c_str()));
    font.setWeight(QFont::Normal);
    font.setPointSize(10);
    i_id->setForeground(Qt::gray);
    i_id->setFont(font);
    int i_id_width = QFontMetrics(font).width(i_id->text());

    if (size_longest_text_per_column[0] < i_type_width)
      size_longest_text_per_column[0] = i_type_width;

    if (size_longest_text_per_column[1] < i_name_width)
      size_longest_text_per_column[1] = i_name_width;

    if (size_longest_text_per_column[2] < i_id_width)
      size_longest_text_per_column[2] = i_id_width;

    q_node_info.append(i_type);
    q_node_info.append(i_name);
    q_node_info.append(i_id);
    bt_model->appendRow(q_node_info);

    // compact model
    QList<QStandardItem*> q_node_info_compact;
    QStandardItem* s_item;

    if (node.hasChildNodes())
    {
      if (!ti.node_name_view.empty())
        s_item =
            new QStandardItem(QString((blanks_compact + ti.node_type_view + " [" + ti.node_name_view + "]").c_str()));
      else
        s_item = new QStandardItem(QString((blanks_compact + ti.node_type_view).c_str()));

      s_item->setForeground(Qt::darkBlue);
      // font.setWeight(QFont::Bold);
      font.setPointSize(10);
    }
    else
    {
      s_item = new QStandardItem(QString((blanks_compact + ti.node_name_view).c_str()));
      s_item->setForeground(Qt::black);
      // font.setWeight(QFont::Normal);
      font.setPointSize(10);
    }

    s_item->setFont(font);
    q_node_info_compact.append(s_item);
    bt_model_compact->appendRow(q_node_info_compact);

    int s_item_width = QFontMetrics(font).width(s_item->text());

    if (size_column_compact[0] < s_item_width)
      size_column_compact[0] = s_item_width;

    map_str_node_to_id_row[ti.getStrID()] = bt_model->rowCount() - 1;
    c_row = bt_model->rowCount();
    if (node.hasChildNodes())
    {
      for (int i = 0; i < node.childNodes().length(); i++)
      {
        convertSubTree(node.childNodes().at(i), level + 1);
      }
    }

    return node;
  }
};

#endif  // TREE_MODEL_H
