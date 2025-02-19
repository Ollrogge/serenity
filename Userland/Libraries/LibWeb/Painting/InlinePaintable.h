/*
 * Copyright (c) 2022, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <LibWeb/Layout/InlineNode.h>
#include <LibWeb/Painting/PaintableBox.h>
#include <LibWeb/Painting/PaintableFragment.h>

namespace Web::Painting {

class InlinePaintable final : public Paintable {
    JS_CELL(InlinePaintable, Paintable);

public:
    static JS::NonnullGCPtr<InlinePaintable> create(Layout::InlineNode const&);

    virtual void paint(PaintContext&, PaintPhase) const override;

    Layout::InlineNode const& layout_node() const;
    auto const& box_model() const { return layout_node().box_model(); }

    CSSPixelRect bounding_rect() const;
    Vector<PaintableFragment> const& fragments() const { return m_fragments; }
    Vector<PaintableFragment>& fragments() { return m_fragments; }

    virtual bool is_inline_paintable() const override { return true; }

    virtual Optional<HitTestResult> hit_test(CSSPixelPoint, HitTestType) const override;

    void set_box_shadow_data(Vector<ShadowData>&& box_shadow_data) { m_box_shadow_data = move(box_shadow_data); }
    Vector<ShadowData> const& box_shadow_data() const { return m_box_shadow_data; }

private:
    InlinePaintable(Layout::InlineNode const&);

    template<typename Callback>
    void for_each_fragment(Callback) const;

    Vector<ShadowData> m_box_shadow_data;
    Vector<PaintableFragment> m_fragments;
};

}
