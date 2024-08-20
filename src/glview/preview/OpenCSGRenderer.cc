/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2009-2011 Clifford Wolf <clifford@clifford.at> and
 *                          Marius Kintel <marius@kintel.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "OpenCSGRenderer.h"
#include "LegacyRendererUtils.h"
#include "linalg.h"
#include "system-gl.h"

#include "Feature.h"
#include "PolySet.h"
#include <memory.h>
#include <utility>

#ifdef ENABLE_OPENCSG

class OpenCSGVBOPrim : public OpenCSG::Primitive {
public:
    OpenCSGVBOPrim(OpenCSG::Operation operation, unsigned int convexity,
                   std::unique_ptr<VertexState> vertex_state)
        : OpenCSG::Primitive(operation, convexity),
          vertex_state(std::move(vertex_state)) {}
    void render() override {
        if (vertex_state != nullptr) {
            vertex_state->draw();
        } else {
            if (OpenSCAD::debug != "") PRINTD("OpenCSGVBOPrim vertex_state was null");
        }
    }

    bool isSelected(){ return isSelected_; }
    void setSelected(bool selected){ isSelected_ = selected; }

private:
    bool isSelected_ {false};
    const std::unique_ptr<VertexState> vertex_state;
};
#endif // ENABLE_OPENCSG

OpenCSGRenderer::OpenCSGRenderer(
        std::shared_ptr<CSGProducts> root_products,
        std::shared_ptr<CSGProducts> highlights_products,
        std::shared_ptr<CSGProducts> background_products)
    : root_products_(std::move(root_products)),
      highlights_products_(std::move(highlights_products)),
      background_products_(std::move(background_products)) {}

void OpenCSGRenderer::prepare(bool /*showfaces*/, bool /*showedges*/,
                              const shaderinfo_t *shaderinfo) {
    //is_selected.clear();

    if (vbo_vertex_products_.empty()) {
        if (root_products_) {
            createCSGVBOProducts(*root_products_, shaderinfo, false, false);
        }
        if (background_products_) {
            createCSGVBOProducts(*background_products_, shaderinfo, false, true);
        }
    }
}

void OpenCSGRenderer::setHighlights(std::shared_ptr<CSGProducts> hi)
{
    is_selected.clear();
    highlights_products_ = hi;

    if(!highlights_products_)
        return;

    for (auto i = 0; i < highlights_products_->products.size(); ++i) {
        const auto &product = highlights_products_->products[i];

        for (const auto &csgobj : product.intersections) {
            if (csgobj.leaf->polyset) {
                is_selected[csgobj.leaf->index] = true;
                std::cout << "Rendering IMMEDIATE: INTERSECTION: " << csgobj.leaf->dump() << std::endl;
                if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_SELECTED) {
                    std::cout << "                       : selected" << std::endl;
                    continue;
                } else if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_IMPACTED) {
                    std::cout << "                       : impacted" << std::endl;
                } else {
                    std::cout << "                       : hightlight" << std::endl;
                }
            }
        }

        for (const auto &csgobj : product.subtractions) {
            if (csgobj.leaf->polyset) {
                is_selected[csgobj.leaf->index] = true;
                std::cout << "Rendering IMMEDIATE: INTERSECTION: " << csgobj.leaf->dump() << std::endl;
                if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_SELECTED) {
                    std::cout << "                       : selected" << std::endl;
                    continue;
                } else if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_IMPACTED) {
                    std::cout << "                       : impacted" << std::endl;
                } else {
                    std::cout << "                       : hightlight" << std::endl;
                }
            }
        }

    }


}

void OpenCSGRenderer::draw(bool /*showfaces*/, bool showedges,
                           const shaderinfo_t *shaderinfo) const {

    if (!shaderinfo && showedges){
        shaderinfo = &getShader();
    }
    renderCSGVBOProducts(showedges, shaderinfo);

    if (!highlights_products_)
        return;

    if(!shaderinfo)
        renderImmediate(highlights_products_);
    return;
}

class OpenCSGPrim : public OpenCSG::Primitive
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OpenCSGPrim(OpenCSG::Operation operation, unsigned int convexity, const OpenCSGRenderer& renderer) :
        OpenCSG::Primitive(operation, convexity), renderer(renderer) { }
    std::shared_ptr<const PolySet> polyset;
    Transform3d m;
    Renderer::csgmode_e csgmode{Renderer::CSGMODE_NONE};

    // This is used by OpenCSG to render depth values
    void render() override {
        if (polyset) {
            glPushMatrix();
            glMultMatrixd(m.data());
            render_surface(*polyset, m);
            glPopMatrix();
        }
    }
private:
    const OpenCSGRenderer& renderer;
};

// Primitive for depth rendering using OpenCSG
std::unique_ptr<OpenCSGPrim> createCSGPrimitive(const CSGChainObject& csgobj, OpenCSG::Operation operation,
                                                bool highlight_mode, bool background_mode, OpenSCADOperator type,
                                                const OpenCSGRenderer &renderer) {
    auto prim = std::make_unique<OpenCSGPrim>(operation, csgobj.leaf->polyset->getConvexity(), renderer);
    prim->polyset = csgobj.leaf->polyset;
    prim->m = csgobj.leaf->matrix;
    if (prim->polyset->getDimension() == 2 && type == OpenSCADOperator::DIFFERENCE) {
        // Scale 2D negative objects 10% in the Z direction to avoid z fighting
        prim->m *= Eigen::Scaling(1.0, 1.0, 1.1);
    }
    prim->csgmode = Renderer::get_csgmode(highlight_mode, background_mode, type);
    return prim;
}


void OpenCSGRenderer::renderImmediate(std::shared_ptr<CSGProducts> products) const
{
    auto shaderinfo = &getShader();
    bool background_mode = false;
    bool highlight_mode = true;
    bool showedges = false;

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-0.1, 0.0);

    for (const auto& product : products->products) {
        // owned_primitives is only for memory management
        std::vector<std::unique_ptr<OpenCSG::Primitive>> owned_primitives;
        std::vector<OpenCSG::Primitive *> primitives;

        for (const auto& csgobj : product.intersections) {
            if (!csgobj.leaf->polyset) continue;

            if (shaderinfo && shaderinfo->type == Renderer::SELECT_RENDERING) {
                int identifier = csgobj.leaf->index;
                GL_CHECKD(glUniform3f(shaderinfo->data.select_rendering.identifier,
                                      ((identifier >> 0) & 0xff) / 255.0f, ((identifier >> 8) & 0xff) / 255.0f,
                                      ((identifier >> 16) & 0xff) / 255.0f));
            }

            const Color4f& c = csgobj.leaf->color;
            csgmode_e csgmode = get_csgmode(highlight_mode, background_mode);

            ColorMode colormode = ColorMode::NONE;
            if (highlight_mode) {
                if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_SELECTED) {
                    colormode = ColorMode::HIGHLIGHT_SELECTED;
                } else if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_IMPACTED) {
                    colormode = ColorMode::HIGHLIGHT_IMPACTED;
                } else {
                    colormode = ColorMode::HIGHLIGHT;
                }

            } else if (background_mode) {
                colormode = ColorMode::BACKGROUND;
            } else {
                colormode = ColorMode::MATERIAL;
            }

            glPushMatrix();
            glMultMatrixd(csgobj.leaf->matrix.data());

            const Color4f color = setColor(colormode, c.data(), shaderinfo);
            if (color[3] == 1.0f) {
                // object is opaque, draw normally
                render_surface(*csgobj.leaf->polyset, csgobj.leaf->matrix, shaderinfo);
            } else {
                // object is transparent, so draw rear faces first.  Issue #1496
                glEnable(GL_CULL_FACE);
                glCullFace(GL_FRONT);
                render_surface(*csgobj.leaf->polyset, csgobj.leaf->matrix, shaderinfo);
                glCullFace(GL_BACK);
                render_surface(*csgobj.leaf->polyset, csgobj.leaf->matrix, shaderinfo);
                glDisable(GL_CULL_FACE);
            }

            glPopMatrix();
        }
        for (const auto& csgobj : product.subtractions) {
            if (!csgobj.leaf->polyset) continue;

            const Color4f& c = csgobj.leaf->color;
            csgmode_e csgmode = get_csgmode(highlight_mode, background_mode, OpenSCADOperator::DIFFERENCE);

            ColorMode colormode = ColorMode::NONE;
            if (highlight_mode) {
                if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_SELECTED) {
                    colormode = ColorMode::HIGHLIGHT_SELECTED;
                } else if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_IMPACTED) {
                    colormode = ColorMode::HIGHLIGHT_IMPACTED;
                } else {
                    colormode = ColorMode::HIGHLIGHT;
                }
            } else if (background_mode) {
                colormode = ColorMode::BACKGROUND;
            } else {
                colormode = ColorMode::CUTOUT;
            }

            (void) setColor(colormode, c.data(), shaderinfo);
            glPushMatrix();
            Transform3d mat = csgobj.leaf->matrix;
            if (csgobj.leaf->polyset->getDimension() == 2) {
                // Scale 2D negative objects 10% in the Z direction to avoid z fighting
                mat *= Eigen::Scaling(1.0, 1.0, 1.1);
            }
            glMultMatrixd(mat.data());
            // negative objects should only render rear faces
            glEnable(GL_CULL_FACE);
            glCullFace(GL_FRONT);
            render_surface(*csgobj.leaf->polyset, csgobj.leaf->matrix, shaderinfo);
            glDisable(GL_CULL_FACE);

            glPopMatrix();
        }

        if (shaderinfo) glUseProgram(0);
        glDepthFunc(GL_LEQUAL);
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
}

// Primitive for drawing using OpenCSG
// Makes a copy of the given VertexState enabling just unlit/uncolored vertex
// rendering
OpenCSGVBOPrim *OpenCSGRenderer::createVBOPrimitive(
    const std::shared_ptr<OpenCSGVertexState> &vertex_state,
    const OpenCSG::Operation operation, const unsigned int convexity) const {
  std::unique_ptr<VertexState> opencsg_vs = std::make_unique<VertexState>(
      vertex_state->drawMode(), vertex_state->drawSize(),
      vertex_state->drawType(), vertex_state->drawOffset(),
      vertex_state->elementOffset(), vertex_state->verticesVBO(),
      vertex_state->elementsVBO());
  // First two glBegin entries are the vertex position calls
  opencsg_vs->glBegin().insert(opencsg_vs->glBegin().begin(),
                               vertex_state->glBegin().begin(),
                               vertex_state->glBegin().begin() + 2);
  // First glEnd entry is the disable vertex position call
  opencsg_vs->glEnd().insert(opencsg_vs->glEnd().begin(),
                             vertex_state->glEnd().begin(),
                             vertex_state->glEnd().begin() + 1);

  return new OpenCSGVBOPrim(operation, convexity, std::move(opencsg_vs));
}

// Turn the CSGProducts into VBOs
// Will create one (temporary) VertexArray and one VBO(+EBO) per product
// The VBO will be utilized to render multiple objects with correct state
// management. In the future, we could use a VBO per primitive and potentially
// reuse VBOs, but that requires some more careful state management.
// Note: This function can be called multiple times for different products.
// Each call will add to vbo_vertex_products_.
void OpenCSGRenderer::createCSGVBOProducts(
    const CSGProducts &products, const Renderer::shaderinfo_t * /*shaderinfo*/,
    bool highlight_mode, bool background_mode) {
  // We need to manage buffers here since we don't have another suitable
  // container for managing the life cycle of VBOs. We're creating one VBO(+EBO)
  // per product.
  std::vector<GLuint> vertices_vbos;
  std::vector<GLuint> elements_vbos;

  size_t vbo_count = products.products.size();
  if (vbo_count) {
    vertices_vbos.resize(vbo_count);
    // Will default to zeroes, so we don't have to keep checking for the
    // Indexing feature
    elements_vbos.resize(vbo_count);
    glGenBuffers(vbo_count, vertices_vbos.data());
    all_vbos_.insert(all_vbos_.end(), vertices_vbos.begin(), vertices_vbos.end());
    if (Feature::ExperimentalVxORenderersIndexing.is_enabled()) {
      glGenBuffers(vbo_count, elements_vbos.data());
      all_vbos_.insert(all_vbos_.end(), elements_vbos.begin(), elements_vbos.end());
    }
  }

#ifdef ENABLE_OPENCSG
  size_t vbo_index = 0;
  for (auto i = 0; i < products.products.size(); ++i) {
    const auto &product = products.products[i];
    const auto vertices_vbo = vertices_vbos[i];
    const auto elements_vbo = elements_vbos[i];

    Color4f last_color;
    std::vector<OpenCSG::Primitive *> primitives;
    std::unique_ptr<std::vector<std::shared_ptr<VertexState>>> vertex_states =
        std::make_unique<std::vector<std::shared_ptr<VertexState>>>();
    VertexArray vertex_array(std::make_unique<OpenCSGVertexStateFactory>(),
                             *(vertex_states.get()), vertices_vbo,
                             elements_vbo);
    vertex_array.addSurfaceData();
    vertex_array.writeSurface();
    if (getShader().progid != 0) {
      add_shader_data(vertex_array);
    } else {
      LOG("Warning: Shader not available");
    }

    size_t num_vertices = 0;
    for (const auto &csgobj : product.intersections) {
      if (csgobj.leaf->polyset) {
        num_vertices += getSurfaceBufferSize(csgobj);
      }
    }
    for (const auto &csgobj : product.subtractions) {
      if (csgobj.leaf->polyset) {
        num_vertices += getSurfaceBufferSize(csgobj);
      }
    }

    vertex_array.allocateBuffers(num_vertices);

    for (const auto &csgobj : product.intersections) {
      if (csgobj.leaf->polyset) {
        const Color4f &c = csgobj.leaf->color;
        csgmode_e csgmode = get_csgmode(highlight_mode, background_mode);

        std::cout << "Rendering: INTERSECTION: " << highlight_mode << " " <<  csgobj.leaf->dump()
                  << " index=" << csgobj.leaf->index
                  << std::endl;

        ColorMode colormode = ColorMode::NONE;
        if(highlight_mode)
        {
            if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_SELECTED) {
                std::cout << "                       : selected" << std::endl;
                colormode = ColorMode::HIGHLIGHT_SELECTED;
            } else if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_IMPACTED) {
                std::cout << "                       : impacted" << std::endl;
                colormode = ColorMode::HIGHLIGHT_IMPACTED;
            } else {
                std::cout << "                       : hightlight" << std::endl;
                colormode = ColorMode::HIGHLIGHT;
            }
        } else if (background_mode) {
            std::cout << "                       : background" << std::endl;
            colormode = ColorMode::BACKGROUND;
        } else {
            std::cout << "                       : material" << std::endl;

            colormode = ColorMode::MATERIAL;
        }

        Color4f color;
        if (getShaderColor(colormode, c, color)) {
          last_color = color;
        }

        add_color(vertex_array, last_color);

        if (color[3] == 1.0f) {
          // object is opaque, draw normally
          create_surface(*csgobj.leaf->polyset, vertex_array, csgmode,
                         csgobj.leaf->matrix, last_color);
          std::shared_ptr<OpenCSGVertexState> surface =
              std::dynamic_pointer_cast<OpenCSGVertexState>(
                  vertex_states->back());
          if (surface != nullptr) {
            is_selected[csgobj.leaf->index] = false;
            surface->setCsgObjectIndex(csgobj.leaf->index);
            primitives.emplace_back(
                createVBOPrimitive(surface, OpenCSG::Intersection,
                                   csgobj.leaf->polyset->getConvexity()));
          }
        } else {
          // object is transparent, so draw rear faces first.  Issue #1496
          std::shared_ptr<VertexState> cull = std::make_shared<VertexState>();
          cull->glBegin().emplace_back([]() {
            GL_TRACE0("glEnable(GL_CULL_FACE)");
            glEnable(GL_CULL_FACE);
          });
          cull->glBegin().emplace_back([]() {
            GL_TRACE0("glCullFace(GL_FRONT)");
            glCullFace(GL_FRONT);
          });
          vertex_states->emplace_back(std::move(cull));

          create_surface(*csgobj.leaf->polyset, vertex_array, csgmode,
                         csgobj.leaf->matrix, last_color);
          std::shared_ptr<OpenCSGVertexState> surface =
              std::dynamic_pointer_cast<OpenCSGVertexState>(
                  vertex_states->back());

          if (surface != nullptr) {
            surface->setCsgObjectIndex(csgobj.leaf->index);
            is_selected[csgobj.leaf->index] = false;

            primitives.emplace_back(
                createVBOPrimitive(surface, OpenCSG::Intersection,
                                   csgobj.leaf->polyset->getConvexity()));

            cull = std::make_shared<VertexState>();
            cull->glBegin().emplace_back([]() {
              GL_TRACE0("glCullFace(GL_BACK)");
              glCullFace(GL_BACK);
            });
            vertex_states->emplace_back(std::move(cull));

            vertex_states->emplace_back(surface);

            cull = std::make_shared<VertexState>();
            cull->glEnd().emplace_back([]() {
              GL_TRACE0("glDisable(GL_CULL_FACE)");
              glDisable(GL_CULL_FACE);
            });
            vertex_states->emplace_back(std::move(cull));
          } else {
            assert(false && "Intersection surface state was nullptr");
          }
        }
      }
    }

    for (const auto &csgobj : product.subtractions) {
      if (csgobj.leaf->polyset) {
        std::cout << "Rendering: SUBTRACTION: " << highlight_mode <<  " "
                  << csgobj.leaf->dump()
                  << " index=" << csgobj.leaf->index
                  << std::endl;

        const Color4f &c = csgobj.leaf->color;
        csgmode_e csgmode = get_csgmode(highlight_mode, background_mode,
                                        OpenSCADOperator::DIFFERENCE);

        ColorMode colormode = ColorMode::NONE;
        if (highlight_mode)
        {
            if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_SELECTED) {
                colormode = ColorMode::HIGHLIGHT_SELECTED;
            } else if (csgobj.flags == CSGNode::Flag::FLAG_HIGHLIGHT_IMPACTED) {
                colormode = ColorMode::HIGHLIGHT_IMPACTED;
            } else {
                colormode = ColorMode::HIGHLIGHT;
            }
        } else if (background_mode) {
            colormode = ColorMode::BACKGROUND;
        } else {
            colormode = ColorMode::MATERIAL;
        }

        Color4f color;
        if (getShaderColor(colormode, c, color)) {
          last_color = color;
        }

        add_color(vertex_array, last_color);

        // negative objects should only render rear faces
        std::shared_ptr<VertexState> cull = std::make_shared<VertexState>();
        cull->glBegin().emplace_back([]() {
          GL_TRACE0("glEnable(GL_CULL_FACE)");
          GL_CHECKD(glEnable(GL_CULL_FACE));
        });
        cull->glBegin().emplace_back([]() {
          GL_TRACE0("glCullFace(GL_FRONT)");
          GL_CHECKD(glCullFace(GL_FRONT));
        });
        vertex_states->emplace_back(std::move(cull));
        Transform3d tmp = csgobj.leaf->matrix;
        if (csgobj.leaf->polyset->getDimension() == 2) {
          // Scale 2D negative objects 10% in the Z direction to avoid z fighting
          tmp *= Eigen::Scaling(1.0, 1.0, 1.1);
        }
        create_surface(*csgobj.leaf->polyset, vertex_array, csgmode, tmp,
                       last_color);
        std::shared_ptr<OpenCSGVertexState> surface =
            std::dynamic_pointer_cast<OpenCSGVertexState>(
                vertex_states->back());
        if (surface != nullptr) {
          surface->setCsgObjectIndex(csgobj.leaf->index);
          is_selected[csgobj.leaf->index] = false;
          primitives.emplace_back(
              createVBOPrimitive(surface, OpenCSG::Subtraction,
                                 csgobj.leaf->polyset->getConvexity()));
        } else {
          assert(false && "Subtraction surface state was nullptr");
        }

        cull = std::make_shared<VertexState>();
        cull->glEnd().emplace_back([]() {
          GL_TRACE0("glDisable(GL_CULL_FACE)");
          GL_CHECKD(glDisable(GL_CULL_FACE));
        });
        vertex_states->emplace_back(std::move(cull));
      }
    }

    if (Feature::ExperimentalVxORenderersIndexing.is_enabled()) {
      GL_TRACE0("glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)");
      GL_CHECKD(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    }
    GL_TRACE0("glBindBuffer(GL_ARRAY_BUFFER, 0)");
    GL_CHECKD(glBindBuffer(GL_ARRAY_BUFFER, 0));

    vertex_array.createInterleavedVBOs();
    vbo_vertex_products_.emplace_back(std::make_unique<OpenCSGVBOProduct>(
        std::move(primitives), std::move(vertex_states) ));
  }
#endif // ENABLE_OPENCSG
}

void OpenCSGRenderer::renderCSGVBOProducts(
    bool showedges, const Renderer::shaderinfo_t *shaderinfo) const {
#ifdef ENABLE_OPENCSG

  for (const auto &product : vbo_vertex_products_) {
    if (product->primitives().size() > 1) {
      GL_CHECKD(OpenCSG::render(product->primitives()));
      GL_TRACE0("glDepthFunc(GL_EQUAL)");
      GL_CHECKD(glDepthFunc(GL_EQUAL));
    }

    if (shaderinfo && shaderinfo->progid) {
      GL_TRACE("glUseProgram(%d)", shaderinfo->progid);
      GL_CHECKD(glUseProgram(shaderinfo->progid));

      if (shaderinfo->type == EDGE_RENDERING && showedges) {
        shader_attribs_enable();
      }
    }

    auto& baseshader = getShader();
    //GL_TRACE("glUseProgram(%d)", baseshader.progid);
    //GL_CHECKD(glUseProgram(baseshader.progid));

    for (const auto &vs : product->states()) {
        if (vs) {
            std::shared_ptr<OpenCSGVertexState> csg_vs =
                    std::dynamic_pointer_cast<OpenCSGVertexState>(vs);

            if (csg_vs) {
                if(is_selected.find(csg_vs->csgObjectIndex()) != is_selected.end()){
                    if(is_selected.at(csg_vs->csgObjectIndex())){
                        std::cout << "This product is selected... " << std::endl;
                        std::cout << "SET SHADER ... COLOR HACK " << std::endl;
                        vs->glBegin().emplace_back([&baseshader]() {
                                std::cout << "RENDERING OBJECT WITH SELECT COLOR" << baseshader.progid << std::endl;
                                GL_CHECKD(glUniform4f(baseshader.data.csg_rendering.color_area, 1.0,0.0,1.0,0.5));
                                GL_CHECKD(glUniform4f(baseshader.data.csg_rendering.color_edge, 0.0,0.0,1.0,0.5));
                        });

                    }
                }
            }

            if (csg_vs) {
                if (shaderinfo && shaderinfo->type == SELECT_RENDERING) {
                    GL_TRACE("glUniform3f(%d, %f, %f, %f)",
                             shaderinfo->data.select_rendering.identifier %
                             (((csg_vs->csgObjectIndex() >> 0) & 0xff) / 255.0f) %
                             (((csg_vs->csgObjectIndex() >> 8) & 0xff) / 255.0f) %
                             (((csg_vs->csgObjectIndex() >> 16) & 0xff) / 255.0f));
                    GL_CHECKD(glUniform3f(
                                  shaderinfo->data.select_rendering.identifier,
                                  ((csg_vs->csgObjectIndex() >> 0) & 0xff) / 255.0f,
                                  ((csg_vs->csgObjectIndex() >> 8) & 0xff) / 255.0f,
                                  ((csg_vs->csgObjectIndex() >> 16) & 0xff) / 255.0f));
                }
            }
            std::shared_ptr<VBOShaderVertexState> shader_vs =
                    std::dynamic_pointer_cast<VBOShaderVertexState>(vs);
            if (!shader_vs || (showedges && shader_vs)) {
                std::cout << "DRAW OBJ ... " << std::endl;
                vs->draw();
            }

            if (csg_vs) {
                if(is_selected.find(csg_vs->csgObjectIndex()) != is_selected.end()){
                    if(is_selected.at(csg_vs->csgObjectIndex())){
                        csg_vs->glBegin().pop_back();
                        std::cout << "REMOVE GLBEIN()" << std::endl;
                    }
                }
            }
        }
    }

    if (shaderinfo && shaderinfo->progid) {
      GL_TRACE0("glUseProgram(0)");
      GL_CHECKD(glUseProgram(0));

      if (shaderinfo->type == EDGE_RENDERING && showedges) {
        shader_attribs_disable();
      }
    }
    GL_TRACE0("glDepthFunc(GL_LEQUAL)");
    GL_CHECKD(glDepthFunc(GL_LEQUAL));
  }
#endif // ENABLE_OPENCSG
}

BoundingBox OpenCSGRenderer::getBoundingBox() const {
  BoundingBox bbox;
  if (root_products_)
    bbox = root_products_->getBoundingBox();
  if (highlights_products_)
    bbox.extend(highlights_products_->getBoundingBox());
  if (background_products_)
    bbox.extend(background_products_->getBoundingBox());

  return bbox;
}
