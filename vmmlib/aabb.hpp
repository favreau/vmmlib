/*
 * Copyright (c) 2006-2016, Visualization and Multimedia Lab,
 *                          University of Zurich <http://vmml.ifi.uzh.ch>,
 *                          Eyescale Software GmbH,
 *                          Blue Brain Project, EPFL
 *
 * This file is part of VMMLib <https://github.com/VMML/vmmlib/>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.  Redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided
 * with the distribution.  Neither the name of the Visualization and Multimedia
 * Lab, University of Zurich nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __VMML__AXIS_ALIGNED_BOUNDING_BOX__HPP__
#define __VMML__AXIS_ALIGNED_BOUNDING_BOX__HPP__

#include <limits>
#include <vmmlib/vector.hpp>

namespace lunchbox
{
template <class T>
void byteswap(T&);
}

namespace vmml
{
template <typename T>
class AABB;
}
namespace staticjson
{
class ObjectHandler;
template <typename U>
void init(vmml::AABB<U>*, ObjectHandler*);
}

namespace vmml
{
/**
 * An axis-aligned bounding box.
 *
 * An empty bounding box has undefined, implementation-specific values. Read
 * operations (getMin(), getMax(), getSize(), isIn(), etc.) have undefined
 * semantics on an empty bounding box. set() and merge() operations will define
 * the bounding box correctly.
 */
template <typename T>
class AABB
{
public:
    /** Create an empty bounding box. */
    AABB();

    /** Create a new bounding box from two corner points */
    AABB(const vector<3, T>& pMin, const vector<3, T>& pMax);

    /** Create a new bounding box from a bounding sphere */
    AABB(const vector<4, T>& sphere);

    /** @return true if the given point is within this bounding box. */
    bool isIn(const vector<3, T>& point) const;

    /** @return true if the given sphere is within this bounding box. */
    bool isIn(const vector<4, T>& sphere) const;

    /**
     * @return true if this bounding box is in front of the given plane
     *         ([normal.x|y|z, d] notation).
     */
    bool isInFront(const vector<4, T>& plane) const;

    /** @return the minimum corner point */
    const vector<3, T>& getMin() const;

    /** @return the maximum corner point */
    const vector<3, T>& getMax() const;

    /** Create the union of this and the given bounding box */
    void merge(const AABB<T>& aabb);

    /** Create the union of this and the given point */
    void merge(const vector<3, T>& point);

    /** Clear this bounding box */
    void reset();

    /** @return true if this bounding box has not been set */
    bool isEmpty() const;

    /** @return true if this and the other bounding box are identical */
    bool operator==(const AABB<T>& other) const;

    /** @return true if this and the other bounding box are not identical */
    bool operator!=(const AABB<T>& other) const;

    /** @return the center of this bounding box */
    vector<3, T> getCenter() const;

    /** @return the size of this bounding box */
    vector<3, T> getSize() const;

    /**
     * Compute the nearest and furthest point of this box relative to the given
     * plane.
     */
    void computeNearFar(const vector<4, T>& plane, vector<3, T>& nearPoint,
                        vector<3, T>& farPoint) const;

    /** @return a bounding box of size one with the minimum point at zero. */
    static AABB<T> makeUnitBox();

#ifdef __INTEL_COMPILER
    // For some reason, ICC does not understand the friend declarations and
    // generates compilation issues. Making the attributes public is not great
    // but it still is consistent with the rest of the vmmlib library (Vectors
    // and Quaternions also expose public class members)
public:
    vector<3, T> _min;
    vector<3, T> _max;
#else
private:
    vector<3, T> _min;
    vector<3, T> _max;
    template <class U>
    friend void lunchbox::byteswap(U&);
    friend void staticjson::init(AABB<float>*, staticjson::ObjectHandler*);
    friend void staticjson::init(AABB<double>*, staticjson::ObjectHandler*);
#endif
};

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const AABB<T>& aabb)
{
    return os << aabb.getMin() << " - " << aabb.getMax();
}

template <typename T>
AABB<T>::AABB()
    : _min(std::numeric_limits<T>::max())
    , _max(std::numeric_limits<T>::min())
{
}

template <>
inline AABB<float>::AABB()
    : _min(std::numeric_limits<float>::max())
    , _max(-std::numeric_limits<float>::max())
{
}

template <>
inline AABB<double>::AABB()
    : _min(std::numeric_limits<double>::max())
    , _max(-std::numeric_limits<double>::max())
{
}

template <typename T>
AABB<T>::AABB(const vector<3, T>& pMin, const vector<3, T>& pMax)
    : _min(vector<3, T>(std::min(pMin[0], pMax[0]), std::min(pMin[1], pMax[1]),
                        std::min(pMin[2], pMax[2])))
    , _max(vector<3, T>(std::max(pMin[0], pMax[0]), std::max(pMin[1], pMax[1]),
                        std::max(pMin[2], pMax[2])))
{
}

template <typename T>
AABB<T>::AABB(const vector<4, T>& sphere)
{
    _max = _min = sphere.getCenter();
    _max += sphere.getRadius();
    _min -= sphere.getRadius();
}

template <typename T>
inline bool AABB<T>::isIn(const vector<3, T>& pos) const
{
    if (pos.x() > _max.x() || pos.y() > _max.y() || pos.z() > _max.z() ||
        pos.x() < _min.x() || pos.y() < _min.y() || pos.z() < _min.z())
    {
        return false;
    }
    return true;
}

template <typename T>
inline bool AABB<T>::isIn(const vector<4, T>& sphere) const
{
    const vector<3, T>& sv = sphere.getCenter();
    sv += sphere.getRadius();
    if (sv.x() > _max.x() || sv.y() > _max.y() || sv.z() > _max.z())
        return false;
    sv -= sphere.getRadius() * 2.0f;
    if (sv.x() < _min.x() || sv.y() < _min.y() || sv.z() < _min.z())
        return false;
    return true;
}

template <typename T>
inline bool AABB<T>::isInFront(const vector<4, T>& plane) const
{
    const auto& extent = getSize() * 0.5f;
    const float d = plane.dot(getCenter());
    const float n = extent.x() * std::abs(plane.x()) +
                    extent.y() * std::abs(plane.y()) +
                    extent.z() * std::abs(plane.z());

    return !(d - n >= 0 || d + n > 0);
}

template <typename T>
inline const vector<3, T>& AABB<T>::getMin() const
{
    return _min;
}

template <typename T>
inline const vector<3, T>& AABB<T>::getMax() const
{
    return _max;
}

template <typename T>
inline bool AABB<T>::operator==(const AABB<T>& other) const
{
    return _min == other._min && _max == other._max;
}

template <typename T>
inline bool AABB<T>::operator!=(const AABB<T>& other) const
{
    return _min != other._min || _max != other._max;
}

template <typename T>
vector<3, T> AABB<T>::getCenter() const
{
    return (_min + _max) * 0.5f;
}

template <typename T>
vector<3, T> AABB<T>::getSize() const
{
    return _max - _min;
}

template <typename T>
void AABB<T>::merge(const AABB<T>& aabb)
{
    const vector<3, T>& min = aabb.getMin();
    const vector<3, T>& max = aabb.getMax();

    if (min.x() < _min.x())
        _min.x() = min.x();
    if (min.y() < _min.y())
        _min.y() = min.y();
    if (min.z() < _min.z())
        _min.z() = min.z();

    if (max.x() > _max.x())
        _max.x() = max.x();
    if (max.y() > _max.y())
        _max.y() = max.y();
    if (max.z() > _max.z())
        _max.z() = max.z();
}

template <typename T>
void AABB<T>::merge(const vector<3, T>& point)
{
    if (point.x() < _min.x())
        _min.x() = point.x();
    if (point.y() < _min.y())
        _min.y() = point.y();
    if (point.z() < _min.z())
        _min.z() = point.z();

    if (point.x() > _max.x())
        _max.x() = point.x();
    if (point.y() > _max.y())
        _max.y() = point.y();
    if (point.z() > _max.z())
        _max.z() = point.z();
}

template <typename T>
inline void AABB<T>::reset()
{
    _min = std::numeric_limits<T>::max();
    _max = -std::numeric_limits<T>::max();
}

template <typename T>
inline bool AABB<T>::isEmpty() const
{
    return _min.x() >= _max.x() || _min.y() >= _max.y() || _min.z() >= _max.x();
}

template <typename T>
inline void AABB<T>::computeNearFar(const vector<4, T>& plane,
                                    vector<3, T>& nearPoint,
                                    vector<3, T>& farPoint) const
{
    for (size_t i = 0; i < 3; ++i)
    {
        if (plane[i] >= 0.0)
        {
            nearPoint[i] = _min[i];
            farPoint[i] = _max[i];
        }
        else
        {
            nearPoint[i] = _max[i];
            farPoint[i] = _min[i];
        }
    }
}

template <typename T>
AABB<T> AABB<T>::makeUnitBox()
{
    return AABB(vector<3, T>(), vector<3, T>(1));
}
}

#endif
