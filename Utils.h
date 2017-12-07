/*
 * Copyright (C) 2017  Florian GOLESTIN
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SOLEIL__UTILS_H_
#define SOLEIL__UTILS_H_

#include <osg/Group>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <string>

#include <iostream>

namespace Soleil {

struct NameVisitor : public osg::NodeVisitor {
  std::string name;
  osg::Node *found = nullptr;

  NameVisitor(osg::NodeVisitor::TraversalMode mode) : osg::NodeVisitor(mode) {}

  void apply(osg::Node &node) override {
    std::cout << "==" << node.getName() << "\n";
    if (node.getName() == name)
      found = &node;
    else {
      traverse(node);
    }
  }
};

osg::Node *GetNodeByName(osg::Node &root, const std::string &name) {
  if (root.getName() == name)
    return &root;
  NameVisitor v(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN);
  v.name = name;

  root.accept(v);
  return v.found;
}

} // Soleil

#endif /* SOLEIL__UTILS_H_ */
