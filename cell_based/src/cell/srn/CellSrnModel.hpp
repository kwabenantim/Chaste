/*

Copyright (c) 2005-2025, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef CELLSRNMODEL_HPP_
#define CELLSRNMODEL_HPP_

#include <vector>
#include "ChasteSerialization.hpp"
#include "ClassIsAbstract.hpp"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include "AbstractSrnModel.hpp"
#include "CellCycleModelOdeHandler.hpp"
#include "SimulationTime.hpp"

typedef boost::shared_ptr<AbstractSrnModel> AbstractSrnModelPtr;

/**
 * SRN model at the cell level, has representation for edges internally. Also
 * contains cell interior (cytoplasmic) SRN. Mostly serves to coordinate between
 * interior/edge SRNs, in case these are specified. Functionality of SRNs is
 * defined in AbstractSrnModel class and user-defined SRN models.
 */
class CellSrnModel : public AbstractSrnModel
{

private:

    /** Needed for serialization. */
    friend class boost::serialization::access;
    /**
     * Archive the SRN model and member variables.
     *
     * @param archive the archive
     * @param version the current version of this class
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractSrnModel>(*this);
        archive & mEdgeSrnModels;
        archive & mpInteriorSrnModel;
    }

    /** Vector of pointers to edge SRN models. */
    std::vector<boost::shared_ptr<AbstractSrnModel>> mEdgeSrnModels;

    /**
     * @typedef abstractsrnmodel_t
     * Type alias for a vector storing pointers to AbstractSrnModel.
     */
    using abstractsrnmodel_t = std::vector<AbstractSrnModelPtr>;

    /** Pointer to interior SRN model. */
    boost::shared_ptr<AbstractSrnModel> mpInteriorSrnModel;

protected:

    /**
     * Copy constructor. Called ONLY when a cell division occurs. See parent class comment for details
     * @param rModel SRN model to be copied
     */
    CellSrnModel(const CellSrnModel &rModel);

public:

    /**
     * @typedef iterator
     * Type alias for non-constant iterator in the SRN model.
     */
    using iterator = abstractsrnmodel_t::iterator;

    /**
     * @typedef const_iterator
     * Type alias for constant iterator in the SRN model.
     */
    using const_iterator = abstractsrnmodel_t::const_iterator;

    /**
     * Returns an iterator pointing to the first element.
     * @return Iterator to the beginning.
     */
    iterator begin() { return mEdgeSrnModels.begin(); }

    /**
     * Returns an iterator pointing one past the last element.
     * @return Iterator to the end.
     */
    iterator end() { return mEdgeSrnModels.end(); }

    /**
     * Returns a constant iterator pointing to the first element.
     * @return Constant iterator to the beginning.
     */
    const_iterator begin() const { return mEdgeSrnModels.begin(); }

    /**
     * Returns a constant iterator pointing one past the last element.
     * @return Constant iterator to the end.
     */
    const_iterator end() const { return mEdgeSrnModels.end(); }

    /**
     * Returns a constant iterator pointing to the first element.
     * @return Constant iterator to the beginning.
     */
    const_iterator cbegin() const { return mEdgeSrnModels.cbegin(); }

    /**
     * Returns a constant iterator pointing one past the last element.
     * @return Constant iterator to the end.
     */
    const_iterator cend() const { return mEdgeSrnModels.cend(); }

    /**
     * Default constuctor.
     */
    CellSrnModel();

    /**
     * Destructor.
     */
    ~CellSrnModel();

    /**
     * Initialize constituent SRN models.
     */
    virtual void Initialise();

    /**
     * Calls SRN model specific behaviour at the time of cell division.
     * All constituent SRNs models (edge and/or interior, if there are any) call their
     * implementation of this method.
     */
    virtual void ResetForDivision();

    /**
     * Simulate SRN models.
     */
    virtual void SimulateToCurrentTime();

    /**
     * Called in Cell::Divide()
     * @return
     */
    virtual AbstractSrnModel* CreateSrnModel();

    /**
     * Adds a vector of SRN models to this cell.
     *
     * @param edgeSrns vector of SRN models. Index of each SRN corresponds to the local edge index
     */
    void AddEdgeSrn(std::vector<AbstractSrnModelPtr> edgeSrns);

    /**
     * Inserts edge SRN at the end of the list.
     *
     * @param pEdgeSrn the edge SRN to be inserted
     */
    void AddEdgeSrnModel(AbstractSrnModelPtr pEdgeSrn);

    /**
     * Get number of edge SRNs.
     *
     * @return the number of edge SRNs
     */
    unsigned GetNumEdgeSrn() const;

    /**
     * Get edge SRN at an index.
     *
     * @param index of the SRN to return
     * @return SRN to be returned
     */
    AbstractSrnModelPtr GetEdgeSrn(unsigned index) const;

    /**
     * Return all edge SRNs.
     *
     * @return vector of SRNs associated to this cell
     */
    const std::vector<AbstractSrnModelPtr>& GetEdges() const;

    /**
     * Set interior SRN.
     *
     * @param pInteriorSrn poiner to an interior SRN model
     */
    void SetInteriorSrnModel(AbstractSrnModelPtr pInteriorSrn);

    /**
     * Returns interior SRN.
     *
     * @return interior SRN
     */
    AbstractSrnModelPtr GetInteriorSrn() const;

    /**
     * Overriden method. We Set mpCell for each SRN contained in this cell.
     *
     * @param pCell pointer to a Cell
     */
    virtual void SetCell(CellPtr pCell);
};

// Declare identifier for the serializer
#include "SerializationExportWrapper.hpp"
CHASTE_CLASS_EXPORT(CellSrnModel)

#endif /* CELLSRNMODEL_HPP_ */
