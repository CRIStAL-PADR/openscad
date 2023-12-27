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

#ifdef _MSC_VER
// Boost conflicts with MPFR under MSVC (google it)
#include <mpfr.h>
#endif

#include "PolySet.h"
#include "PolySetUtils.h"
#include "printutils.h"
#include "Feature.h"

#include "CGALRenderer.h"
#ifdef ENABLE_CGAL
#include "CGAL_OGL_VBOPolyhedron.h"
#include "CGALHybridPolyhedron.h"
#endif
#ifdef ENABLE_MANIFOLD
#include "ManifoldGeometry.h"
#endif

//#include "Preferences.h"

CGALRenderer::CGALRenderer(const std::shared_ptr<const class Geometry>& geom)
  : last_render_state(Feature::ExperimentalVxORenderers.is_enabled()) // FIXME: this is temporary to make switching between renderers seamless.
{
  this->addGeometry(geom);
  PRINTD("CGALRenderer::CGALRenderer() -> createPolyhedrons()");
#ifdef ENABLE_CGAL
  if (!this->nefPolyhedrons.empty() && this->polyhedrons.empty()) createPolyhedrons();
#endif
}

void CGALRenderer::addGeometry(const std::shared_ptr<const Geometry>& geom)
{
  if (const auto geomlist = std::dynamic_pointer_cast<const GeometryList>(geom)) {
    for (const auto& item : geomlist->getChildren()) {
      this->addGeometry(item.second);
    }
  } else if (const auto ps = std::dynamic_pointer_cast<const PolySet>(geom)) {
    assert(ps->getDimension() == 3);
    // We need to tessellate here, in case the generated PolySet contains concave polygons
    // See tests/data/scad/3D/features/polyhedron-concave-test.scad
    this->polysets.push_back(PolySetUtils::tessellate_faces(*ps));
  } else if (const auto poly = std::dynamic_pointer_cast<const Polygon2d>(geom)) {
    this->polysets.push_back(std::shared_ptr<const PolySet>(poly->tessellate()));
#ifdef ENABLE_CGAL
  } else if (const auto new_N = std::dynamic_pointer_cast<const CGAL_Nef_polyhedron>(geom)) {
    assert(new_N->getDimension() == 3);
    if (!new_N->isEmpty()) {
      this->nefPolyhedrons.push_back(new_N);
    }
  } else if (const auto hybrid = std::dynamic_pointer_cast<const CGALHybridPolyhedron>(geom)) {
    // TODO(ochafik): Implement rendering of CGAL_HybridMesh (CGAL::Surface_mesh) instead.
    this->polysets.push_back(hybrid->toPolySet());
#endif
#ifdef ENABLE_MANIFOLD
  } else if (const auto mani = std::dynamic_pointer_cast<const ManifoldGeometry>(geom)) {
    this->polysets.push_back(mani->toPolySet());
#endif
  } else {
    assert(false && "unsupported geom in CGALRenderer");
  }
}

CGALRenderer::~CGALRenderer()
{
  if (polyset_vertices_vbo) {
    glDeleteBuffers(1, &polyset_vertices_vbo);
  }
  if (polyset_elements_vbo) {
    glDeleteBuffers(1, &polyset_elements_vbo);
  }
}

#ifdef ENABLE_CGAL
void CGALRenderer::createPolyhedrons()
{
  PRINTD("createPolyhedrons");
  this->polyhedrons.clear();

  if (!Feature::ExperimentalVxORenderers.is_enabled()) {
    for (const auto& N : this->nefPolyhedrons) {
      auto p = new CGAL_OGL_Polyhedron(*this->colorscheme);
      CGAL::OGL::Nef3_Converter<CGAL_Nef_polyhedron3>::convert_to_OGLPolyhedron(*N->p3, p);
      // CGAL_NEF3_MARKED_FACET_COLOR <- CGAL_FACE_BACK_COLOR
      // CGAL_NEF3_UNMARKED_FACET_COLOR <- CGAL_FACE_FRONT_COLOR
      p->init();
      this->polyhedrons.push_back(std::shared_ptr<CGAL_OGL_Polyhedron>(p));
    }
  } else {
    for (const auto& N : this->nefPolyhedrons) {
      auto p = new CGAL_OGL_VBOPolyhedron(*this->colorscheme);
      CGAL::OGL::Nef3_Converter<CGAL_Nef_polyhedron3>::convert_to_OGLPolyhedron(*N->p3, p);
      // CGAL_NEF3_MARKED_FACET_COLOR <- CGAL_FACE_BACK_COLOR
      // CGAL_NEF3_UNMARKED_FACET_COLOR <- CGAL_FACE_FRONT_COLOR
      p->init();
      this->polyhedrons.push_back(std::shared_ptr<CGAL_OGL_Polyhedron>(p));
    }
  }
  PRINTD("createPolyhedrons() end");
}
#endif

// Overridden from Renderer
void CGALRenderer::setColorScheme(const ColorScheme& cs)
{
  PRINTD("setColorScheme");
  Renderer::setColorScheme(cs);
  colormap[ColorMode::CGAL_FACE_2D_COLOR] = ColorMap::getColor(cs, RenderColor::CGAL_FACE_2D_COLOR);
  colormap[ColorMode::CGAL_EDGE_2D_COLOR] = ColorMap::getColor(cs, RenderColor::CGAL_EDGE_2D_COLOR);
#ifdef ENABLE_CGAL
  this->polyhedrons.clear(); // Mark as dirty
#endif
  PRINTD("setColorScheme done");
}

void CGALRenderer::createPolySets()
{
  PRINTD("createPolySets() polyset");

  polyset_states.clear();

  glGenBuffers(1, &polyset_vertices_vbo);
  if (Feature::ExperimentalVxORenderersIndexing.is_enabled()) {
    glGenBuffers(1, &polyset_elements_vbo);
  }

  VertexArray vertex_array(std::make_unique<VertexStateFactory>(), polyset_states, polyset_vertices_vbo, polyset_elements_vbo);

  vertex_array.addEdgeData();
  vertex_array.addSurfaceData();


  size_t num_vertices = 0;
  if (this->polysets.size()) {
    for (const auto& polyset : this->polysets) {
      num_vertices += getSurfaceBufferSize(*polyset);
      num_vertices += getEdgeBufferSize(*polyset);
    }
  }

  vertex_array.allocateBuffers(num_vertices);

  for (const auto& polyset : this->polysets) {
    Color4f color;

    PRINTD("polysets");
    if (polyset->getDimension() == 2) {
      PRINTD("2d polysets");
      vertex_array.writeEdge();

      std::shared_ptr<VertexState> init_state = std::make_shared<VertexState>();
      init_state->glEnd().emplace_back([]() {
        GL_TRACE0("glDisable(GL_LIGHTING)");
        GL_CHECKD(glDisable(GL_LIGHTING));
      });
      polyset_states.emplace_back(std::move(init_state));

      // Create 2D polygons
      getColor(ColorMode::CGAL_FACE_2D_COLOR, color);
      this->create_polygons(*polyset, vertex_array, CSGMODE_NONE, Transform3d::Identity(), color);

      std::shared_ptr<VertexState> edge_state = std::make_shared<VertexState>();
      edge_state->glBegin().emplace_back([]() {
        GL_TRACE0("glDisable(GL_DEPTH_TEST)");
        GL_CHECKD(glDisable(GL_DEPTH_TEST));
      });
      edge_state->glBegin().emplace_back([]() {
        GL_TRACE0("glLineWidth(2)");
        GL_CHECKD(glLineWidth(2));
      });
      polyset_states.emplace_back(std::move(edge_state));

      // Create 2D edges
      getColor(ColorMode::CGAL_EDGE_2D_COLOR, color);
      this->create_edges(*polyset, vertex_array, CSGMODE_NONE, Transform3d::Identity(), color);

      std::shared_ptr<VertexState> end_state = std::make_shared<VertexState>();
      end_state->glBegin().emplace_back([]() {
        GL_TRACE0("glEnable(GL_DEPTH_TEST)");
        GL_CHECKD(glEnable(GL_DEPTH_TEST));
      });
      polyset_states.emplace_back(std::move(end_state));
    } else {
      PRINTD("3d polysets");
      vertex_array.writeSurface();

      // Create 3D polygons
      getColor(ColorMode::MATERIAL, color);
      this->create_surface(*polyset, vertex_array, CSGMODE_NORMAL, Transform3d::Identity(), color);
    }
  }

  if (this->polysets.size()) {
    if (Feature::ExperimentalVxORenderersDirect.is_enabled() || Feature::ExperimentalVxORenderersPrealloc.is_enabled()) {
      if (Feature::ExperimentalVxORenderersIndexing.is_enabled()) {
        GL_TRACE0("glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)");
        GL_CHECKD(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
      }
      GL_TRACE0("glBindBuffer(GL_ARRAY_BUFFER, 0)");
      GL_CHECKD(glBindBuffer(GL_ARRAY_BUFFER, 0));
    }

    vertex_array.createInterleavedVBOs();
  }
}

void CGALRenderer::prepare(bool /*showfaces*/, bool /*showedges*/, const shaderinfo_t * /*shaderinfo*/)
{
  PRINTD("prepare()");
  if (!polyset_states.size()) createPolySets();
#ifdef ENABLE_CGAL
  if (!this->nefPolyhedrons.empty() &&
      (this->polyhedrons.empty() || Feature::ExperimentalVxORenderers.is_enabled() != last_render_state)) // FIXME: this is temporary to make switching between renderers seamless.
    createPolyhedrons();
#endif

  PRINTD("prepare() end");
}

void CGALRenderer::draw(bool showfaces, bool showedges, const shaderinfo_t * /*shaderinfo*/) const
{
  PRINTD("draw()");
  if (!Feature::ExperimentalVxORenderers.is_enabled()) {
    for (const auto& polyset : this->polysets) {
      PRINTD("draw() polyset");
      if (polyset->getDimension() == 2) {
        // Draw 2D polygons
        glDisable(GL_LIGHTING);
        setColor(ColorMode::CGAL_FACE_2D_COLOR);

        for (const auto& polygon : polyset->indices) {
          glBegin(GL_POLYGON);
          for (const auto& ind : polygon) {
            Vector3d p=polyset->vertices[ind];		  
            glVertex3d(p[0], p[1], 0);
          }
          glEnd();
        }

        // Draw 2D edges
        glDisable(GL_DEPTH_TEST);

        glLineWidth(2);
        setColor(ColorMode::CGAL_EDGE_2D_COLOR);
        this->render_edges(*polyset, CSGMODE_NONE);
        glEnable(GL_DEPTH_TEST);
      } else {
        // Draw 3D polygons
        setColor(ColorMode::MATERIAL);
        this->render_surface(*polyset, CSGMODE_NORMAL, Transform3d::Identity(), nullptr);
      }
    }
  } else {
    // grab current state to restore after
    GLfloat current_point_size, current_line_width;
    GLboolean origVertexArrayState = glIsEnabled(GL_VERTEX_ARRAY);
    GLboolean origNormalArrayState = glIsEnabled(GL_NORMAL_ARRAY);
    GLboolean origColorArrayState = glIsEnabled(GL_COLOR_ARRAY);

    GL_CHECKD(glGetFloatv(GL_POINT_SIZE, &current_point_size));
    GL_CHECKD(glGetFloatv(GL_LINE_WIDTH, &current_line_width));

    for (const auto& polyset : polyset_states) {
      if (polyset) polyset->draw();
    }

    // restore states
    GL_TRACE("glPointSize(%d)", current_point_size);
    GL_CHECKD(glPointSize(current_point_size));
    GL_TRACE("glLineWidth(%d)", current_line_width);
    GL_CHECKD(glLineWidth(current_line_width));

    if (!origVertexArrayState) glDisableClientState(GL_VERTEX_ARRAY);
    if (!origNormalArrayState) glDisableClientState(GL_NORMAL_ARRAY);
    if (!origColorArrayState) glDisableClientState(GL_COLOR_ARRAY);
  }

#ifdef ENABLE_CGAL
  for (const auto& p : this->getPolyhedrons()) {
    *const_cast<bool *>(&last_render_state) = Feature::ExperimentalVxORenderers.is_enabled(); // FIXME: this is temporary to make switching between renderers seamless.
    if (showfaces) p->set_style(SNC_BOUNDARY);
    else p->set_style(SNC_SKELETON);
    p->draw(showfaces && showedges);
  }
#endif

  PRINTD("draw() end");
}

BoundingBox CGALRenderer::getBoundingBox() const
{
  BoundingBox bbox;

#ifdef ENABLE_CGAL
  for (const auto& p : this->getPolyhedrons()) {
    CGAL::Bbox_3 cgalbbox = p->bbox();
    bbox.extend(BoundingBox(
                  Vector3d(cgalbbox.xmin(), cgalbbox.ymin(), cgalbbox.zmin()),
                  Vector3d(cgalbbox.xmax(), cgalbbox.ymax(), cgalbbox.zmax())));
  }
#endif
  for (const auto& ps : this->polysets) {
    bbox.extend(ps->getBoundingBox());
  }
  return bbox;
}
double calculateLinePointDistance(const Vector3d &l1, const Vector3d &l2, const Vector3d &pt, double & dist_lat) {
    Vector3d d = (l2 - l1).normalized();
    dist_lat = (pt-l1).dot(d);
    return (l1 + d * dist_lat-pt).norm();
}

double calculateLineLineDistance(const Vector3d &l1b, const Vector3d &l1e, const Vector3d &l2b, const Vector3d &l2e, double &dist_lat)
{
	double d;
	Vector3d v1=l1e-l1b;
	Vector3d v2=l2e-l2b;
	Vector3d n=v1.cross(v2);
	if(n.norm() == 0) {
		return calculateLinePointDistance(l1b,l1e,l2b,d);
	}
	double t=n.norm();
	n.normalize();
    	d=n.dot(l1b-l2b);
	dist_lat=(v2.cross(n)).dot(l2b-l1b)/t;
	return d;
}

std::vector<SelectedObject> CGALRenderer::findModelObject(Vector3d near_pt, Vector3d far_pt,int mouse_x, int mouse_y, double tolerance) {
  std::vector<SelectedObject> results;
  double dist_near;
  double dist_nearest=NAN;
  Vector3d pt1_nearest;
  Vector3d pt2_nearest;
  for (const std::shared_ptr<const PolySet>& ps : this->polysets) {
    for(const Vector3d &pt: ps->vertices) {
      double dist_pt= calculateLinePointDistance(near_pt, far_pt, pt, dist_near);
      if(dist_pt < tolerance  ) {
        if(isnan(dist_nearest) || dist_near < dist_nearest)
        {
          dist_nearest=dist_near;
          pt1_nearest=pt;
        }	  
      }
    }
  }
  if(!isnan(dist_nearest)) {
    SelectedObject obj;
    obj.type = SELECTION_POINT;
    obj.p1=pt1_nearest;
    results.push_back(obj);
    return results;
  }
  for (const std::shared_ptr<const PolySet>& ps : this->polysets) {
    for(const auto &pol : ps->indices) {
	int n = pol.size();
        for(int i=0;i < n;i++ )
	{
	  int ind1=pol[i];
	  int ind2=pol[(i+1)%n];
	  double dist_lat;
          double dist_norm= fabs(calculateLineLineDistance(ps->vertices[ind1], ps->vertices[ind2], near_pt, far_pt,dist_lat));
          if(dist_lat >= 0 && dist_lat <= 1 && dist_norm < tolerance  ) {
	      dist_nearest=dist_lat;
	      pt1_nearest=ps->vertices[ind1];
	      pt2_nearest=ps->vertices[ind2];
	  }
        }	  
      }
   }

  if(!isnan(dist_nearest)) {
    SelectedObject obj;
    obj.type = SELECTION_LINE;
    obj.p1=pt1_nearest;
    obj.p2=pt2_nearest;
    results.push_back(obj);
    return results;
  }
  return results;
}
