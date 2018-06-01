/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TDecCu.cpp
    \brief    CU decoder class
*/

#include "TDecCu.h"
#include "TLibCommon/TComRdCost.h"
#include "TLibCommon/TComTU.h"
#include "TLibCommon/TComPrediction.h"

void funcaoArquivo (TComDataCU * CU);
void funcaoArquivo2 (TComDataCU * CU);
int TreeDepth0(float *x);
int TreeDepth1(float *x);
int TreeDepth2(float *x);
int TreeDepth3(float *x);

//! \ingroup TLibDecoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCu::TDecCu()
{
  m_ppcYuvResi = NULL;
  m_ppcYuvReco = NULL;
  m_ppcCU      = NULL;
}

TDecCu::~TDecCu()
{
}

Void TDecCu::init( TDecEntropy* pcEntropyDecoder, TComTrQuant* pcTrQuant, TComPrediction* pcPrediction)
{
  m_pcEntropyDecoder  = pcEntropyDecoder;
  m_pcTrQuant         = pcTrQuant;
  m_pcPrediction      = pcPrediction;
}

/**
 \param    uiMaxDepth      total number of allowable depth
 \param    uiMaxWidth      largest CU width
 \param    uiMaxHeight     largest CU height
 \param    chromaFormatIDC chroma format
 */
Void TDecCu::create( UInt uiMaxDepth, UInt uiMaxWidth, UInt uiMaxHeight, ChromaFormat chromaFormatIDC )
{
  m_uiMaxDepth = uiMaxDepth+1;

  m_ppcYuvResi = new TComYuv*[m_uiMaxDepth-1];
  m_ppcYuvReco = new TComYuv*[m_uiMaxDepth-1];
  m_ppcCU      = new TComDataCU*[m_uiMaxDepth-1];

  UInt uiNumPartitions;
  for ( UInt ui = 0; ui < m_uiMaxDepth-1; ui++ )
  {
    uiNumPartitions = 1<<( ( m_uiMaxDepth - ui - 1 )<<1 );
    UInt uiWidth  = uiMaxWidth  >> ui;
    UInt uiHeight = uiMaxHeight >> ui;

    m_ppcYuvResi[ui] = new TComYuv;    m_ppcYuvResi[ui]->create( uiWidth, uiHeight, chromaFormatIDC );
    m_ppcYuvReco[ui] = new TComYuv;    m_ppcYuvReco[ui]->create( uiWidth, uiHeight, chromaFormatIDC );
    m_ppcCU     [ui] = new TComDataCU; m_ppcCU     [ui]->create( chromaFormatIDC, uiNumPartitions, uiWidth, uiHeight, true, uiMaxWidth >> (m_uiMaxDepth - 1) );
  }

  m_bDecodeDQP = false;
  m_IsChromaQpAdjCoded = false;

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster(m_uiMaxDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uiMaxDepth );

  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uiMaxDepth );
}

Void TDecCu::destroy()
{
  for ( UInt ui = 0; ui < m_uiMaxDepth-1; ui++ )
  {
    m_ppcYuvResi[ui]->destroy(); delete m_ppcYuvResi[ui]; m_ppcYuvResi[ui] = NULL;
    m_ppcYuvReco[ui]->destroy(); delete m_ppcYuvReco[ui]; m_ppcYuvReco[ui] = NULL;
    m_ppcCU     [ui]->destroy(); delete m_ppcCU     [ui]; m_ppcCU     [ui] = NULL;
  }

  delete [] m_ppcYuvResi; m_ppcYuvResi = NULL;
  delete [] m_ppcYuvReco; m_ppcYuvReco = NULL;
  delete [] m_ppcCU     ; m_ppcCU      = NULL;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** 
 Parse a CTU.
 \param    pCtu                      [in/out] pointer to CTU data structure
 \param    isLastCtuOfSliceSegment   [out]    true, if last CTU of the slice segment
 */
Void TDecCu::decodeCtu( TComDataCU* pCtu, Bool& isLastCtuOfSliceSegment )
{
  if ( pCtu->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  if ( pCtu->getSlice()->getUseChromaQpAdj() )
  {
    setIsChromaQpAdjCoded(true);
  }

  // start from the top level CU
  xDecodeCU( pCtu, 0, 0, isLastCtuOfSliceSegment);
}

/** 
 Decoding process for a CTU.
 \param    pCtu                      [in/out] pointer to CTU data structure
 */
Void TDecCu::decompressCtu( TComDataCU* pCtu )
{
  xDecompressCU( pCtu, 0,  0 );
  funcaoArquivo(pCtu);
  //funcaoArquivo2(pCtu);
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! decode end-of-slice flag
Bool TDecCu::xDecodeSliceEnd( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  UInt uiIsLastCtuOfSliceSegment;

  if (pcCU->isLastSubCUOfCtu(uiAbsPartIdx))
  {
    m_pcEntropyDecoder->decodeTerminatingBit( uiIsLastCtuOfSliceSegment );
  }
  else
  {
    uiIsLastCtuOfSliceSegment=0;
  }

  return uiIsLastCtuOfSliceSegment>0;
}

//! decode CU block recursively
Void TDecCu::xDecodeCU( TComDataCU*const pcCU, const UInt uiAbsPartIdx, const UInt uiDepth, Bool &isLastCtuOfSliceSegment)
{
  TComPic* pcPic        = pcCU->getPic();
  const TComSPS &sps    = pcPic->getPicSym()->getSPS();
  const TComPPS &pps    = pcPic->getPicSym()->getPPS();
  const UInt maxCuWidth = sps.getMaxCUWidth();
  const UInt maxCuHeight= sps.getMaxCUHeight();
  UInt uiCurNumParts    = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);
  UInt uiQNumParts      = uiCurNumParts>>2;


  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (maxCuWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (maxCuHeight>>uiDepth) - 1;

  if( ( uiRPelX < sps.getPicWidthInLumaSamples() ) && ( uiBPelY < sps.getPicHeightInLumaSamples() ) )
  {
    m_pcEntropyDecoder->decodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
  }
  else
  {
    bBoundary = true;
  }
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() ) ) || bBoundary )
  {
    UInt uiIdx = uiAbsPartIdx;
    if( uiDepth == pps.getMaxCuDQPDepth() && pps.getUseDQP())
    {
      setdQPFlag(true);
      pcCU->setQPSubParts( pcCU->getRefQP(uiAbsPartIdx), uiAbsPartIdx, uiDepth ); // set QP to default QP
    }

    if( uiDepth == pps.getMaxCuChromaQpAdjDepth() && pcCU->getSlice()->getUseChromaQpAdj() )
    {
      setIsChromaQpAdjCoded(true);
    }

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

      if ( !isLastCtuOfSliceSegment && ( uiLPelX < sps.getPicWidthInLumaSamples() ) && ( uiTPelY < sps.getPicHeightInLumaSamples() ) )
      {
        xDecodeCU( pcCU, uiIdx, uiDepth+1, isLastCtuOfSliceSegment );
      }
      else
      {
        pcCU->setOutsideCUPart( uiIdx, uiDepth+1 );
      }

      uiIdx += uiQNumParts;
    }
    if( uiDepth == pps.getMaxCuDQPDepth() && pps.getUseDQP())
    {
      if ( getdQPFlag() )
      {
        UInt uiQPSrcPartIdx = uiAbsPartIdx;
        pcCU->setQPSubParts( pcCU->getRefQP( uiQPSrcPartIdx ), uiAbsPartIdx, uiDepth ); // set QP to default QP
      }
    }
    return;
  }

  if( uiDepth <= pps.getMaxCuDQPDepth() && pps.getUseDQP())
  {
    setdQPFlag(true);
    pcCU->setQPSubParts( pcCU->getRefQP(uiAbsPartIdx), uiAbsPartIdx, uiDepth ); // set QP to default QP
  }

  if( uiDepth <= pps.getMaxCuChromaQpAdjDepth() && pcCU->getSlice()->getUseChromaQpAdj() )
  {
    setIsChromaQpAdjCoded(true);
  }

  if (pps.getTransquantBypassEnableFlag())
  {
    m_pcEntropyDecoder->decodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx, uiDepth );
  }

  // decode CU mode and the partition size
  if( !pcCU->getSlice()->isIntra())
  {
    m_pcEntropyDecoder->decodeSkipFlag( pcCU, uiAbsPartIdx, uiDepth );
  }


  if( pcCU->isSkipped(uiAbsPartIdx) )
  {
    m_ppcCU[uiDepth]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_0 );
    m_ppcCU[uiDepth]->copyInterPredInfoFrom( pcCU, uiAbsPartIdx, REF_PIC_LIST_1 );
    TComMvField cMvFieldNeighbours[MRG_MAX_NUM_CANDS << 1]; // double length for mv of both lists
    UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
    Int numValidMergeCand = 0;
    for( UInt ui = 0; ui < m_ppcCU[uiDepth]->getSlice()->getMaxNumMergeCand(); ++ui )
    {
      uhInterDirNeighbours[ui] = 0;
    }
    m_pcEntropyDecoder->decodeMergeIndex( pcCU, 0, uiAbsPartIdx, uiDepth );
    UInt uiMergeIndex = pcCU->getMergeIndex(uiAbsPartIdx);
    m_ppcCU[uiDepth]->getInterMergeCandidates( 0, 0, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand, uiMergeIndex );
    pcCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeIndex], uiAbsPartIdx, 0, uiDepth );

    TComMv cTmpMv( 0, 0 );
    for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
    {
      if ( pcCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
      {
        pcCU->setMVPIdxSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, 0, uiDepth);
        pcCU->setMVPNumSubParts( 0, RefPicList( uiRefListIdx ), uiAbsPartIdx, 0, uiDepth);
        pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvd( cTmpMv, SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
        pcCU->getCUMvField( RefPicList( uiRefListIdx ) )->setAllMvField( cMvFieldNeighbours[ 2*uiMergeIndex + uiRefListIdx ], SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
      }
    }
    xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, isLastCtuOfSliceSegment );
    return;
  }

  m_pcEntropyDecoder->decodePredMode( pcCU, uiAbsPartIdx, uiDepth );
  m_pcEntropyDecoder->decodePartSize( pcCU, uiAbsPartIdx, uiDepth );

  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
  {
    m_pcEntropyDecoder->decodeIPCMInfo( pcCU, uiAbsPartIdx, uiDepth );

    if(pcCU->getIPCMFlag(uiAbsPartIdx))
    {
      xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, isLastCtuOfSliceSegment );
      return;
    }
  }

  // prediction mode ( Intra : direction mode, Inter : Mv, reference idx )
  m_pcEntropyDecoder->decodePredInfo( pcCU, uiAbsPartIdx, uiDepth, m_ppcCU[uiDepth]);

  // Coefficient decoding
  Bool bCodeDQP = getdQPFlag();
  Bool isChromaQpAdjCoded = getIsChromaQpAdjCoded();
  m_pcEntropyDecoder->decodeCoeff( pcCU, uiAbsPartIdx, uiDepth, bCodeDQP, isChromaQpAdjCoded );
  setIsChromaQpAdjCoded( isChromaQpAdjCoded );
  setdQPFlag( bCodeDQP );
  xFinishDecodeCU( pcCU, uiAbsPartIdx, uiDepth, isLastCtuOfSliceSegment );

}

Void TDecCu::xFinishDecodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, Bool &isLastCtuOfSliceSegment)
{
  if(  pcCU->getSlice()->getPPS()->getUseDQP())
  {
    pcCU->setQPSubParts( getdQPFlag()?pcCU->getRefQP(uiAbsPartIdx):pcCU->getCodedQP(), uiAbsPartIdx, uiDepth ); // set QP
  }

  if (pcCU->getSlice()->getUseChromaQpAdj() && !getIsChromaQpAdjCoded())
  {
    pcCU->setChromaQpAdjSubParts( pcCU->getCodedChromaQpAdj(), uiAbsPartIdx, uiDepth ); // set QP
  }

  isLastCtuOfSliceSegment = xDecodeSliceEnd( pcCU, uiAbsPartIdx );
}

Void TDecCu::xDecompressCU( TComDataCU* pCtu, UInt uiAbsPartIdx,  UInt uiDepth )
{
  TComPic* pcPic = pCtu->getPic();
  TComSlice * pcSlice = pCtu->getSlice();
  const TComSPS &sps=*(pcSlice->getSPS());

  Bool bBoundary = false;
  UInt uiLPelX   = pCtu->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (sps.getMaxCUWidth()>>uiDepth)  - 1;
  UInt uiTPelY   = pCtu->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (sps.getMaxCUHeight()>>uiDepth) - 1;

  if( ( uiRPelX >= sps.getPicWidthInLumaSamples() ) || ( uiBPelY >= sps.getPicHeightInLumaSamples() ) )
  {
    bBoundary = true;
  }

  if( ( ( uiDepth < pCtu->getDepth( uiAbsPartIdx ) ) && ( uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() ) ) || bBoundary )
  {
    UInt uiNextDepth = uiDepth + 1;
    UInt uiQNumParts = pCtu->getTotalNumPart() >> (uiNextDepth<<1);
    UInt uiIdx = uiAbsPartIdx;
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++ )
    {
      uiLPelX = pCtu->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      uiTPelY = pCtu->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];

      if( ( uiLPelX < sps.getPicWidthInLumaSamples() ) && ( uiTPelY < sps.getPicHeightInLumaSamples() ) )
      {
        xDecompressCU(pCtu, uiIdx, uiNextDepth );
      }

      uiIdx += uiQNumParts;
    }
    return;
  }

  // Residual reconstruction
  m_ppcYuvResi[uiDepth]->clear();

  m_ppcCU[uiDepth]->copySubCU( pCtu, uiAbsPartIdx, uiDepth );

  switch( m_ppcCU[uiDepth]->getPredictionMode(0) )
  {
    case MODE_INTER:
      xReconInter( m_ppcCU[uiDepth], uiDepth );
      break;
    case MODE_INTRA:
      xReconIntraQT( m_ppcCU[uiDepth], uiDepth );
      break;
    default:
      assert(0);
      break;
  }

#ifdef DEBUG_STRING
  const PredMode predMode=m_ppcCU[uiDepth]->getPredictionMode(0);
  if (DebugOptionList::DebugString_Structure.getInt()&DebugStringGetPredModeMask(predMode))
  {
    PartSize eSize=m_ppcCU[uiDepth]->getPartitionSize(0);
    std::ostream &ss(std::cout);

    ss <<"###: " << (predMode==MODE_INTRA?"Intra   ":"Inter   ") << partSizeToString[eSize] << " CU at " << m_ppcCU[uiDepth]->getCUPelX() << ", " << m_ppcCU[uiDepth]->getCUPelY() << " width=" << UInt(m_ppcCU[uiDepth]->getWidth(0)) << std::endl;
  }
#endif

  if ( m_ppcCU[uiDepth]->isLosslessCoded(0) && (m_ppcCU[uiDepth]->getIPCMFlag(0) == false))
  {
    xFillPCMBuffer(m_ppcCU[uiDepth], uiDepth);
  }

  xCopyToPic( m_ppcCU[uiDepth], pcPic, uiAbsPartIdx, uiDepth );
}

Void TDecCu::xReconInter( TComDataCU* pcCU, UInt uiDepth )
{

  // inter prediction
  m_pcPrediction->motionCompensation( pcCU, m_ppcYuvReco[uiDepth] );

#ifdef DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTER);
  if (DebugOptionList::DebugString_Pred.getInt()&debugPredModeMask)
  {
    printBlockToStream(std::cout, "###inter-pred: ", *(m_ppcYuvReco[uiDepth]));
  }
#endif

  // inter recon
  xDecodeInterTexture( pcCU, uiDepth );

#ifdef DEBUG_STRING
  if (DebugOptionList::DebugString_Resi.getInt()&debugPredModeMask)
  {
    printBlockToStream(std::cout, "###inter-resi: ", *(m_ppcYuvResi[uiDepth]));
  }
#endif

  // clip for only non-zero cbp case
  if  ( pcCU->getQtRootCbf( 0) )
  {
    m_ppcYuvReco[uiDepth]->addClip( m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth], 0, pcCU->getWidth( 0 ), pcCU->getSlice()->getSPS()->getBitDepths() );
  }
  else
  {
    m_ppcYuvReco[uiDepth]->copyPartToPartYuv( m_ppcYuvReco[uiDepth],0, pcCU->getWidth( 0 ),pcCU->getHeight( 0 ));
  }
#ifdef DEBUG_STRING
  if (DebugOptionList::DebugString_Reco.getInt()&debugPredModeMask)
  {
    printBlockToStream(std::cout, "###inter-reco: ", *(m_ppcYuvReco[uiDepth]));
  }
#endif

}


Void
TDecCu::xIntraRecBlk(       TComYuv*    pcRecoYuv,
                            TComYuv*    pcPredYuv,
                            TComYuv*    pcResiYuv,
                      const ComponentID compID,
                            TComTU     &rTu)
{
  if (!rTu.ProcessComponentSection(compID))
  {
    return;
  }
  const Bool       bIsLuma = isLuma(compID);


  TComDataCU *pcCU = rTu.getCU();
  const TComSPS &sps=*(pcCU->getSlice()->getSPS());
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();

  const TComRectangle &tuRect  =rTu.getRect(compID);
  const UInt uiWidth           = tuRect.width;
  const UInt uiHeight          = tuRect.height;
  const UInt uiStride          = pcRecoYuv->getStride (compID);
        Pel* piPred            = pcPredYuv->getAddr( compID, uiAbsPartIdx );
  const ChromaFormat chFmt     = rTu.GetChromaFormat();

  if (uiWidth != uiHeight)
  {
    //------------------------------------------------

    //split at current level if dividing into square sub-TUs

    TComTURecurse subTURecurse(rTu, false, TComTU::VERTICAL_SPLIT, true, compID);

    //recurse further
    do
    {
      xIntraRecBlk(pcRecoYuv, pcPredYuv, pcResiYuv, compID, subTURecurse);
    } while (subTURecurse.nextSection(rTu));

    //------------------------------------------------

    return;
  }

  const UInt uiChPredMode  = pcCU->getIntraDir( toChannelType(compID), uiAbsPartIdx );
  const UInt partsPerMinCU = 1<<(2*(sps.getMaxTotalCUDepth() - sps.getLog2DiffMaxMinCodingBlockSize()));
  const UInt uiChCodedMode = (uiChPredMode==DM_CHROMA_IDX && !bIsLuma) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
  const UInt uiChFinalMode = ((chFmt == CHROMA_422)       && !bIsLuma) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

  //===== init availability pattern =====
  Bool  bAboveAvail = false;
  Bool  bLeftAvail  = false;

  const Bool bUseFilteredPredictions=TComPrediction::filteringIntraReferenceSamples(compID, uiChFinalMode, uiWidth, uiHeight, chFmt, pcCU->getSlice()->getSPS()->getDisableIntraReferenceSmoothing());

#ifdef DEBUG_STRING
  std::ostream &ss(std::cout);
#endif

  DEBUG_STRING_NEW(sTemp)
  m_pcPrediction->initAdiPatternChType( rTu, bAboveAvail, bLeftAvail, compID, bUseFilteredPredictions  DEBUG_STRING_PASS_INTO(sTemp) );


  //===== get prediction signal =====

  m_pcPrediction->predIntraAng( compID,   uiChFinalMode, 0 /* Decoder does not have an original image */, 0, piPred, uiStride, rTu, bAboveAvail, bLeftAvail, bUseFilteredPredictions );

#ifdef DEBUG_STRING
  ss << sTemp;
#endif

  //===== inverse transform =====
  Pel*      piResi            = pcResiYuv->getAddr( compID, uiAbsPartIdx );
  TCoeff*   pcCoeff           = pcCU->getCoeff(compID) + rTu.getCoefficientOffset(compID);//( uiNumCoeffInc * uiAbsPartIdx );

  const QpParam cQP(*pcCU, compID);


  DEBUG_STRING_NEW(sDebug);
#ifdef DEBUG_STRING
  const Int debugPredModeMask=DebugStringGetPredModeMask(MODE_INTRA);
  std::string *psDebug=(DebugOptionList::DebugString_InvTran.getInt()&debugPredModeMask) ? &sDebug : 0;
#endif

  if (pcCU->getCbf(uiAbsPartIdx, compID, rTu.GetTransformDepthRel()) != 0)
  {
    m_pcTrQuant->invTransformNxN( rTu, compID, piResi, uiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO(psDebug) );
  }
  else
  {
    for (UInt y = 0; y < uiHeight; y++)
    {
      for (UInt x = 0; x < uiWidth; x++)
      {
        piResi[(y * uiStride) + x] = 0;
      }
    }
  }

#ifdef DEBUG_STRING
  if (psDebug)
  {
    ss << (*psDebug);
  }
#endif

  //===== reconstruction =====
  const UInt uiRecIPredStride  = pcCU->getPic()->getPicYuvRec()->getStride(compID);

  const Bool useCrossComponentPrediction = isChroma(compID) && (pcCU->getCrossComponentPredictionAlpha(uiAbsPartIdx, compID) != 0);
  const Pel* pResiLuma  = pcResiYuv->getAddr( COMPONENT_Y, uiAbsPartIdx );
  const Int  strideLuma = pcResiYuv->getStride( COMPONENT_Y );

        Pel* pPred      = piPred;
        Pel* pResi      = piResi;
        Pel* pReco      = pcRecoYuv->getAddr( compID, uiAbsPartIdx );
        Pel* pRecIPred  = pcCU->getPic()->getPicYuvRec()->getAddr( compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiAbsPartIdx );


#ifdef DEBUG_STRING
  const Bool bDebugPred=((DebugOptionList::DebugString_Pred.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
  const Bool bDebugResi=((DebugOptionList::DebugString_Resi.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
  const Bool bDebugReco=((DebugOptionList::DebugString_Reco.getInt()&debugPredModeMask) && DEBUG_STRING_CHANNEL_CONDITION(compID));
  if (bDebugPred || bDebugResi || bDebugReco)
  {
    ss << "###: " << "CompID: " << compID << " pred mode (ch/fin): " << uiChPredMode << "/" << uiChFinalMode << " absPartIdx: " << rTu.GetAbsPartIdxTU() << std::endl;
  }
#endif

  const Int clipbd = sps.getBitDepth(toChannelType(compID));
#if O0043_BEST_EFFORT_DECODING
  const Int bitDepthDelta = sps.getStreamBitDepth(toChannelType(compID)) - clipbd;
#endif

  if( useCrossComponentPrediction )
  {
    TComTrQuant::crossComponentPrediction( rTu, compID, pResiLuma, piResi, piResi, uiWidth, uiHeight, strideLuma, uiStride, uiStride, true );
  }

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
#ifdef DEBUG_STRING
    if (bDebugPred || bDebugResi || bDebugReco)
    {
      ss << "###: ";
    }

    if (bDebugPred)
    {
      ss << " - pred: ";
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        ss << pPred[ uiX ] << ", ";
      }
    }
    if (bDebugResi)
    {
      ss << " - resi: ";
    }
#endif

    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
#ifdef DEBUG_STRING
      if (bDebugResi)
      {
        ss << pResi[ uiX ] << ", ";
      }
#endif
#if O0043_BEST_EFFORT_DECODING
      pReco    [ uiX ] = ClipBD( rightShiftEvenRounding<Pel>(pPred[ uiX ] + pResi[ uiX ], bitDepthDelta), clipbd );
#else
      pReco    [ uiX ] = ClipBD( pPred[ uiX ] + pResi[ uiX ], clipbd );
#endif
      pRecIPred[ uiX ] = pReco[ uiX ];
    }
#ifdef DEBUG_STRING
    if (bDebugReco)
    {
      ss << " - reco: ";
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )
      {
        ss << pReco[ uiX ] << ", ";
      }
    }

    if (bDebugPred || bDebugResi || bDebugReco)
    {
      ss << "\n";
    }
#endif
    pPred     += uiStride;
    pResi     += uiStride;
    pReco     += uiStride;
    pRecIPred += uiRecIPredStride;
  }
}


Void
TDecCu::xReconIntraQT( TComDataCU* pcCU, UInt uiDepth )
{
  if (pcCU->getIPCMFlag(0))
  {
    xReconPCM( pcCU, uiDepth );
    return;
  }
  const UInt numChType = pcCU->getPic()->getChromaFormat()!=CHROMA_400 ? 2 : 1;
  for (UInt chType=CHANNEL_TYPE_LUMA; chType<numChType; chType++)
  {
    const ChannelType chanType=ChannelType(chType);
    const Bool NxNPUHas4Parts = ::isChroma(chanType) ? enable4ChromaPUsInIntraNxNCU(pcCU->getPic()->getChromaFormat()) : true;
    const UInt uiInitTrDepth = ( pcCU->getPartitionSize(0) != SIZE_2Nx2N && NxNPUHas4Parts ? 1 : 0 );

    TComTURecurse tuRecurseCU(pcCU, 0);
    TComTURecurse tuRecurseWithPU(tuRecurseCU, false, (uiInitTrDepth==0)?TComTU::DONT_SPLIT : TComTU::QUAD_SPLIT);

    do
    {
      xIntraRecQT( m_ppcYuvReco[uiDepth], m_ppcYuvReco[uiDepth], m_ppcYuvResi[uiDepth], chanType, tuRecurseWithPU );
    } while (tuRecurseWithPU.nextSection(tuRecurseCU));
  }
}



/** Function for deriving reconstructed PU/CU chroma samples with QTree structure
 * \param pcRecoYuv pointer to reconstructed sample arrays
 * \param pcPredYuv pointer to prediction sample arrays
 * \param pcResiYuv pointer to residue sample arrays
 * \param chType    texture channel type (luma/chroma)
 * \param rTu       reference to transform data
 *
 \ This function derives reconstructed PU/CU chroma samples with QTree recursive structure
 */

Void
TDecCu::xIntraRecQT(TComYuv*    pcRecoYuv,
                    TComYuv*    pcPredYuv,
                    TComYuv*    pcResiYuv,
                    const ChannelType chType,
                    TComTU     &rTu)
{
  UInt uiTrDepth    = rTu.GetTransformDepthRel();
  TComDataCU *pcCU  = rTu.getCU();
  UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  UInt uiTrMode     = pcCU->getTransformIdx( uiAbsPartIdx );
  if( uiTrMode == uiTrDepth )
  {
    if (isLuma(chType))
    {
      xIntraRecBlk( pcRecoYuv, pcPredYuv, pcResiYuv, COMPONENT_Y,  rTu );
    }
    else
    {
      const UInt numValidComp=getNumberValidComponents(rTu.GetChromaFormat());
      for(UInt compID=COMPONENT_Cb; compID<numValidComp; compID++)
      {
        xIntraRecBlk( pcRecoYuv, pcPredYuv, pcResiYuv, ComponentID(compID), rTu );
      }
    }
  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      xIntraRecQT( pcRecoYuv, pcPredYuv, pcResiYuv, chType, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));
  }
}

Void TDecCu::xCopyToPic( TComDataCU* pcCU, TComPic* pcPic, UInt uiZorderIdx, UInt uiDepth )
{
  UInt uiCtuRsAddr = pcCU->getCtuRsAddr();

  m_ppcYuvReco[uiDepth]->copyToPicYuv  ( pcPic->getPicYuvRec (), uiCtuRsAddr, uiZorderIdx );

  return;
}

Void TDecCu::xDecodeInterTexture ( TComDataCU* pcCU, UInt uiDepth )
{

  TComTURecurse tuRecur(pcCU, 0, uiDepth);

  for(UInt ch=0; ch<pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID=ComponentID(ch);
    DEBUG_STRING_OUTPUT(std::cout, debug_reorder_data_inter_token[compID])

    m_pcTrQuant->invRecurTransformNxN ( compID, m_ppcYuvResi[uiDepth], tuRecur );
  }

  DEBUG_STRING_OUTPUT(std::cout, debug_reorder_data_inter_token[MAX_NUM_COMPONENT])
}

/** Function for deriving reconstructed luma/chroma samples of a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiPartIdx part index
 * \param piPCM pointer to PCM code arrays
 * \param piReco pointer to reconstructed sample arrays
 * \param uiStride stride of reconstructed sample arrays
 * \param uiWidth CU width
 * \param uiHeight CU height
 * \param compID colour component ID
 * \returns Void
 */
Void TDecCu::xDecodePCMTexture( TComDataCU* pcCU, const UInt uiPartIdx, const Pel *piPCM, Pel* piReco, const UInt uiStride, const UInt uiWidth, const UInt uiHeight, const ComponentID compID)
{
        Pel* piPicReco         = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiPartIdx);
  const UInt uiPicStride       = pcCU->getPic()->getPicYuvRec()->getStride(compID);
  const TComSPS &sps           = *(pcCU->getSlice()->getSPS());
  const UInt uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));

  for(UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for(UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      piReco[uiX] = (piPCM[uiX] << uiPcmLeftShiftBit);
      piPicReco[uiX] = piReco[uiX];
    }
    piPCM += uiWidth;
    piReco += uiStride;
    piPicReco += uiPicStride;
  }
}

/** Function for reconstructing a PCM mode CU.
 * \param pcCU pointer to current CU
 * \param uiDepth CU Depth
 * \returns Void
 */
Void TDecCu::xReconPCM( TComDataCU* pcCU, UInt uiDepth )
{
  const UInt maxCuWidth     = pcCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt maxCuHeight    = pcCU->getSlice()->getSPS()->getMaxCUHeight();
  for (UInt ch=0; ch < pcCU->getPic()->getNumberValidComponents(); ch++)
  {
    const ComponentID compID = ComponentID(ch);
    const UInt width  = (maxCuWidth >>(uiDepth+m_ppcYuvResi[uiDepth]->getComponentScaleX(compID)));
    const UInt height = (maxCuHeight>>(uiDepth+m_ppcYuvResi[uiDepth]->getComponentScaleY(compID)));
    const UInt stride = m_ppcYuvResi[uiDepth]->getStride(compID);
    Pel * pPCMChannel = pcCU->getPCMSample(compID);
    Pel * pRecChannel = m_ppcYuvReco[uiDepth]->getAddr(compID);
    xDecodePCMTexture( pcCU, 0, pPCMChannel, pRecChannel, stride, width, height, compID );
  }
}

/** Function for filling the PCM buffer of a CU using its reconstructed sample array
 * \param pCU   pointer to current CU
 * \param depth CU Depth
 */
Void TDecCu::xFillPCMBuffer(TComDataCU* pCU, UInt depth)
{
  const ChromaFormat format = pCU->getPic()->getChromaFormat();
  const UInt numValidComp   = getNumberValidComponents(format);
  const UInt maxCuWidth     = pCU->getSlice()->getSPS()->getMaxCUWidth();
  const UInt maxCuHeight    = pCU->getSlice()->getSPS()->getMaxCUHeight();

  for (UInt componentIndex = 0; componentIndex < numValidComp; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    const UInt width  = maxCuWidth  >> (depth + getComponentScaleX(component, format));
    const UInt height = maxCuHeight >> (depth + getComponentScaleY(component, format));

    Pel *source      = m_ppcYuvReco[depth]->getAddr(component, 0, width);
    Pel *destination = pCU->getPCMSample(component);

    const UInt sourceStride = m_ppcYuvReco[depth]->getStride(component);

    for (Int line = 0; line < height; line++)
    {
      for (Int column = 0; column < width; column++)
      {
        destination[column] = source[column];
      }

      source      += sourceStride;
      destination += width;
    }
  }
}

//! \}

void funcaoArquivo (TComDataCU * CU){
    FILE *fp, *fp2;
    int i,length;

    float reducao = 0.6;
    
    int *depth;
    int *predMode;
    int *partSize;
    int *skipFlag;
    int *qp;
    int *transformIdx;
    int *mergeflag;
    int *mergeindex;
    int *intraDirLuma;
    int *intraDirCroma;
    int *interDir;  
    int *explicitRdpcmModeY;
    int *explicitRdpcmModeCb;
    int *explicitRdpcmModeCr;
    int *CbfY;
    int *CbfCb;
    int *CbfCr;
    int *apiMVPIdx;
    int *apiMVPNum;

    
    int TotalSize;
    TotalSize = CU->getTotalNumPart();
    
    if(TotalSize != 256)
    {
        printf("Cuidado, problema de inconsistencia da variavel CU->getTotalNumPart()\n");
    }
    
    depth         = (int *) malloc(TotalSize*sizeof(int));
    predMode      = (int *) malloc(TotalSize*sizeof(int));
    partSize      = (int *) malloc(TotalSize*sizeof(int));
    skipFlag      = (int *) malloc(TotalSize*sizeof(int));
    qp            = (int *) malloc(TotalSize*sizeof(int));
    transformIdx  = (int *) malloc(TotalSize*sizeof(int));
    mergeflag     = (int *) malloc(TotalSize*sizeof(int));
    mergeindex    = (int *) malloc(TotalSize*sizeof(int));
    intraDirLuma  = (int *) malloc(TotalSize*sizeof(int));
    intraDirCroma = (int *) malloc(TotalSize*sizeof(int));
    interDir      = (int *) malloc(TotalSize*sizeof(int));
    explicitRdpcmModeY= (int *) malloc(TotalSize*sizeof(int));
    explicitRdpcmModeCb= (int *) malloc(TotalSize*sizeof(int));
    explicitRdpcmModeCr= (int *) malloc(TotalSize*sizeof(int));
    CbfY		= (int *) malloc(TotalSize*sizeof(int));
    CbfCb 			= (int *) malloc(TotalSize*sizeof(int));
    CbfCr 			= (int *) malloc(TotalSize*sizeof(int));
    apiMVPIdx		= (int *) malloc(TotalSize*sizeof(int));
    apiMVPNum		= (int *) malloc(TotalSize*sizeof(int));


    fp = fopen ("traceMatlab.csv","a");
    fp2 = fopen ("trace2.csv","a");

    //fprintf(fp,"%d,%d,%d,",CU->getCtuRsAddr(),CU->getCUPelX(),CU->getCUPelY());



    i=0;
    length=0;
    while (i < CU->getTotalNumPart()){
        depth[length]           = (int) CU->getDepth(i);
        predMode[length]        = (int) CU->getPredictionMode(i);
        partSize[length]        = (int) CU->getPartitionSize(i);
        skipFlag[length]        = (int) CU->getSkipFlag(i);
        qp[length]              = (int) CU->getQP(i);
        transformIdx[length]     = (int) CU->getTransformIdx(i);
        mergeflag[length]       = (int) CU->getMergeFlag(i);
        mergeindex[length]      = (int) CU->getMergeIndex(i);
        intraDirLuma[length]    = (int) CU->getIntraDir(toChannelType(ComponentID(0)),i);
        intraDirCroma[length]   = (int) CU->getIntraDir(toChannelType(ComponentID(1)),i);
        interDir[length]        = (int) CU->getInterDir(i);
        explicitRdpcmModeY[length]= (int) CU->getExplicitRdpcmMode(ComponentID(COMPONENT_Y), i);
        explicitRdpcmModeCb[length]= (int) CU->getExplicitRdpcmMode(ComponentID(COMPONENT_Cb), i);
        explicitRdpcmModeCr[length]= (int) CU->getExplicitRdpcmMode(ComponentID(COMPONENT_Cr), i);
        CbfY[length] 			= (int) CU->getCbf(i, ComponentID(COMPONENT_Y));
        CbfCb[length] 			= (int) CU->getCbf(i, ComponentID(COMPONENT_Cb));
        CbfCr[length] 			= (int) CU->getCbf(i, ComponentID(COMPONENT_Cr));
        apiMVPIdx[length]		= (int) CU->getMVPIdx(REF_PIC_LIST_0, i);
        apiMVPNum[length]		= (int) CU->getMVPNum(REF_PIC_LIST_0, i);

        //i = i + 1;

        if(depth[length] == 3){
          i = i + ((16>>depth[length]) * (16>>depth[length]));
          i = i + ((16>>depth[length]) * (16>>depth[length]));
          i = i + ((16>>depth[length]) * (16>>depth[length]));
        }
        i = i + ((16>>depth[length]) * (16>>depth[length]));
        length++;
    }

    //fprintf(fp,"%d,%d,%d\n",length,CU->getPic()->getPOC(),CU->getSlice()->getSliceType());   


//Parte para a criação das tabelas de treinamento da arvore de decisão
    //fprintf(fp, "Address,PosX,PosY,Lengh,Slice,Type,Depth,PredMode,PartSize,SkipFlag,QP,TransformIdx,mergeflag,mergeindex,intraDirLuma,intraDirCroma,interDir,explicitRdpcmModeY,explicitRdpcmModeCb,explicitRdpcmModeCr,CbfY,CbfCb,CbfCr,apiMVPIdx,apiMVPNum \n");

    if(depth[0] == 0){
      float x[5];

      x[0]= CU->getCtuRsAddr();
      x[1]= CU->getCUPelX();
      x[2]= CU->getCUPelY(); 

      x[3]= qp[0];
      x[4]= reducao;

      fprintf(fp, "%d", TreeDepth0(x));
      fprintf(fp2, "%d", depth[0]);

    }else if(depth[0] == 1){
            float x[11];

            x[0]= CU->getCtuRsAddr();
            x[1]= CU->getCUPelX();
            x[2]= CU->getCUPelY();
            x[3]= CU->getSlice()->getSliceType(); 

            x[4]= predMode[0];    
            x[5]= qp[0];    
            x[6]= interDir[0];
            x[7]= CbfY[0];    
            x[8]= CbfCb[0];    
            x[9]= CbfCr[0];
            x[10]= reducao;

            fprintf(fp, "%d", TreeDepth1(x));
            fprintf(fp2, "%d", depth[0]);


          }else if(depth[0] == 2){
                  float x[12];

                  x[0]= CU->getCtuRsAddr();
                  x[1]= CU->getCUPelX();
                  x[2]= CU->getCUPelY();
                  x[3]= CU->getSlice()->getSliceType(); 
                  x[4]= predMode[0];    
                  x[5]= qp[0];    
                  x[6]= transformIdx[0];    
                  x[7]= intraDirLuma[0];    
                  x[8]= intraDirCroma[0];    
                  x[9]= interDir[0];
                  x[10]= CbfCr[0]; 
                  x[11]= reducao;

                  fprintf(fp, "%d", TreeDepth2(x));
                  fprintf(fp2, "%d", depth[0]);


                }else if(depth[0] == 3){
                        float x[8];

                        x[0]= CU->getCtuRsAddr();
                        x[1]= CU->getCUPelX();
                        x[2]= CU->getCUPelY();
                        x[3]= CU->getSlice()->getSliceType(); 
                        x[4]= skipFlag[0];  
                        x[5]= qp[0];
                        x[6]= interDir[0];
                        x[7]= reducao;

                        fprintf(fp, "%d", TreeDepth3(x));
                        fprintf(fp2, "%d", depth[0]);


                }

  	for(i=1;i<length;i++){
        if(depth[i] == 0){
              float x[5];

              x[0]= CU->getCtuRsAddr();
              x[1]= CU->getCUPelX();
              x[2]= CU->getCUPelY(); 

              x[3]= qp[i];
              x[4]= reducao;

              fprintf(fp, "%s%d", ",",TreeDepth0(x));
              fprintf(fp2,"%s%d", ",",depth[i]);


            }else if(depth[i] == 1){
                    float x[11];

                    x[0]= CU->getCtuRsAddr();
                    x[1]= CU->getCUPelX();
                    x[2]= CU->getCUPelY();
                    x[3]= CU->getSlice()->getSliceType(); 

                    x[4]= predMode[i];    
                    x[5]= qp[i];    
                    x[6]= interDir[i];
                    x[7]= CbfY[i];    
                    x[8]= CbfCb[i];    
                    x[9]= CbfCr[i];
                    x[10]= reducao;

                    fprintf(fp, "%s%d", ",",TreeDepth1(x));
                    fprintf(fp2,"%s%d", ",",depth[i]);


                  }else if(depth[i] == 2){
                          float x[12];

                          x[0]= CU->getCtuRsAddr();
                          x[1]= CU->getCUPelX();
                          x[2]= CU->getCUPelY();
                          x[3]= CU->getSlice()->getSliceType(); 
                          x[4]= predMode[i];    
                          x[5]= qp[i];    
                          x[6]= transformIdx[i];    
                          x[7]= intraDirLuma[i];    
                          x[8]= intraDirCroma[i];    
                          x[9]= interDir[i];
                          x[10]= CbfCr[i]; 
                          x[11]= reducao;

                          fprintf(fp, "%s%d", ",",TreeDepth2(x));
                          fprintf(fp2,"%s%d", ",",depth[i]);


                        }else if(depth[i] == 3){
                                float x[8];

                                x[0]= CU->getCtuRsAddr();
                                x[1]= CU->getCUPelX();
                                x[2]= CU->getCUPelY();
                                x[3]= CU->getSlice()->getSliceType(); 
                                x[4]= skipFlag[i];  
                                x[5]= qp[i];
                                x[6]= interDir[i];
                                x[7]= reducao;

                                fprintf(fp, "%s%d", ",",TreeDepth3(x));
                                fprintf(fp2,"%s%d", ",",depth[i]);


                        }

    }
    fprintf(fp,"\n");
    fprintf(fp2, "\n");

    fclose(fp);
    fclose(fp2);
   
    free(depth);
    free(predMode);
    free(partSize);
    free(skipFlag);
    free(qp);
    free(transformIdx);
    free(mergeflag);
    free(mergeindex);
    free(intraDirLuma);
    free(intraDirCroma);
    free(interDir);
    free(explicitRdpcmModeY);
    free(explicitRdpcmModeCb);
    free(explicitRdpcmModeCr);
    free(CbfY);
    free(CbfCb);
    free(CbfCr);
    free(apiMVPIdx);
    free(apiMVPNum);
    
}

void funcaoArquivo2 (TComDataCU * CU){
    FILE *fp;
    int i,length;
    
    int *depth;
    
    int TotalSize;
    TotalSize = CU->getTotalNumPart();
    
    if(TotalSize != 256)
    {
        printf("Cuidado, problema de inconsistencia da variavel CU->getTotalNumPart()\n");
    }
    
    depth = (int *) malloc(TotalSize*sizeof(int));
    
    fp = fopen ("trace2.csv","a");

    i=0;
    length=0;
    while (i < CU->getTotalNumPart()){
        depth[length] = (int) CU->getDepth(i);        
        //i = i + 1;
        if(depth[length] == 3){
          i = i + ((16>>depth[length]) * (16>>depth[length]));
          i = i + ((16>>depth[length]) * (16>>depth[length]));
          i = i + ((16>>depth[length]) * (16>>depth[length]));
        }
        i = i + ((16>>depth[length]) * (16>>depth[length]));
        length++;
    }
    
    fprintf(fp,"%d",depth[0]);

    for(i=1;i<length;i++){
        fprintf(fp,"%s%d", ",",depth[i]);
    }
    fprintf(fp,"\n");
    fclose(fp);
 
    free(depth);    
}

int TreeDepth0(float *x){
  if(x[0] > 999) return 1;
  else{
      if(x[1] > 2496) return 1 ;
      else{
          if(x[3] > 48) return 1 ;
          else{
              if(x[4] <= 0.3) return 0 ;
              else{
                  if(x[0] > 219){
                      if(x[3] <= 30) return 1 ;
                      else{
                          if(x[3] <= 32) return 0 ;
                          else{
                              if(x[3] <= 38) return 1 ;
                              else return 0 ;
                          }
                      }            
                  }else{
                      if(x[3] > 32) return 0 ;
                      else{
                          if(x[1] > 1216) return 1 ;
                          else{
                              if(x[3] <= 21) return 1 ;
                              else{
                                  if(x[2] > 512) return 0 ;
                                  else{
                                      if (x[0] > 172) return 1 ;
                                      else{
                                          if(x[4] <= 0.8) return 0 ;
                                          else return 1 ;
                                      }
                                  }
                              }
                          }
                      }
                  }
              }           
          }
      }                                    
  }

}

int TreeDepth1(float *x){
  if(x[3] >= 2){  
      if(x[2] <= 0)   return 0 ;
      else  return 1;
  }else{  
      if(x[4] >= 2)  return -1;
      else{  
          if(x[10] <= 0.25){  
              if(x[5] >= 51)  return 0 ;
              else{  
                  if(x[1] <= 2496)  return -1; 
                  else{ 
                      if(x[5] <= 31)  return 1;
                      else return -1;
                  }
              }
          }else{ 
              if(x[6] <= 1){  
                  if(x[7] >= 1){  
                      if(x[5] >= 42)  return 1;
                      else{  
                          if(x[5] <= 22)  return 0 ;
                          else  return -1;
                      }
                  }else{  
                      if(x[5] <= 46){ 
                          if(x[5] <= 25)  return -1;
                          else   return 1;
                      }else { 
                          if(x[10] >= 0.8)  return 1;
                          else{
                              if(x[5] <= 49)   return 0 ;
                              else{ 
                                  if(x[5] <= 50)   return 1;
                                  else  return 0 ;
                              }
                          }
                      }
                  }
              }else{ 
                  if(x[1] >= 3136){ 
                      if(x[5] <= 44)   return 1;
                      else{ 
                          if(x[10] <= 0.4)   return 0 ;
                          else{ 
                              if(x[10] <= 0.85)    return 0 ;
                              else    return 1;
                          }
                      }
                  }else{  
                      if(x[0] >= 1377){  
                          if(x[5] <= 43)    return 1;
                          else{ 
                              if(x[10] <= 0.8)   return 0 ;
                              else   return 1;
                          }
                      }else{ 
                          if(x[5] >= 43){
                              if(x[5] <= 42)  return 1;
                              else{ 
                                  if(x[5] <= 50)  return 1;
                                  else{
                                      if(x[9] <= 0)  return 0 ;
                                      else  return 1;
                                  }
                              }
                          }else{ 
                              if(x[10] <= 0.5)   return -1;
                              else{  
                                  if(x[5] >= 33)   return -1;
                                  else{ 
                                      if(x[5] <= 22){  
                                          if(x[6] <= 2)  return -1;
                                          else 
                                             if(x[10] <= 0.6)  return 0 ;
                                              else 
                                                  if(x[0] <= 356)  return 0 ;
                                                  else  return 1;
                                      }else{ 
                                          if(x[8] >= 7)  return 1;
                                          else{ 
                                              if(x[0] >= 287)  return 0 ;
                                              else{
                                                  if(x[2] >= 512)  return -1;
                                                  else{ 
                                                      if(x[10] <= 0.65)   return -1;
                                                      else   return 0 ;
                                                  }
                                              }
                                          }
                                      }
                                  }
                              }
                          }                            
                      }                            
                  }                                
              }                                    
          }                                        
      }                                            
  }
}

int TreeDepth2(float *x){
  if(x[3] >= 2)  return 1;
  else{ 
      if(x[11] >= 0.5){ 
          if(x[5] <= 29){  
              if(x[11] <= 0.7)  return -1;
              else{   
                  if(x[0] <= 333){  
                      if(x[2] >= 576)  return -1;
                      else{ 
                          if(x[1] >= 1984)  return 0;
                          else{
                              if(x[0] <= 187)  return -1;
                              else   return 0;
                          }
                      }
                  }else{ 
                      if( x[11] >= 0.9)  return 0;
                      else{  
                          if(x[5] >= 23)  return -1;
                          else{  
                              if(x[5] <= 17)  return -1;
                              else  return 0;
                          }
                      }
                  }
              }
          }else{  
              if(x[5] <= 44){  
                  if(x[0] <= 522 ){ 
                      if(x[2] >= 576)  return -1;
                      else{  
                          if(x[5] >= 38)  return 1;
                          else{ 
                              if(x[5] <= 31)  return 1;
                              else   return -1;
                          }
                      }
                  }else{ 
                      if(x[5] >= 44) { 
                          if(x[0] <= 1459)  return 1;
                          else   return -1;
                      }else{  
                          if(x[5] <= 30)  return 1;
                          else{   
                              if(x[0] >= 2058)  return 1;
                              else{   
                                  if(x[2] <= 1984)  return 1;
                                  else   return 0;
                              }
                          }
                      }
                  }
              }else{ 
                  if(x[11] >= 0.85){  
                      if(x[5] <= 50)  return 1;
                      else{  
                          if(x[6] <= 0)  return 0;
                          else{   
                              if(x[2] <= 1024)  return 1;
                              else   return 0;
                          }
                      }
                  }else{   
                      if(x[5] <= 50){  
                          if(x[11] <= 0.6) { 
                              if(x[2] <= 1024)  return 0;
                              else   return -1;
                          }else{  
                              if(x[11] <= 0.65)  return 0;
                              else{   
                                  if(x[8] >= 26)  return 1;
                                  else{   
                                      if(x[5] <= 49)  return 0;
                                      else   return 1;
                                  }
                              }
                          }
                      }else{   
                          if(x[4] >= 1)  return 0;
                          else{   
                              if(x[2] >= 2112)  return 0;
                              else{   
                                  if(x[1] >= 4032)  return 0;
                                  else{   
                                      if(x[11] >= 0.8)  return 0;
                                      else{   
                                          if(x[2] <= 896){  
                                              if(x[0] >= 546)  return 0;
                                              else{    
                                                  if(x[11] <= 0.5)  return -1;
                                                  else   return 0;
                                              }
                                          }else{   
                                              if(x[0] <= 1023)  return -1;
                                              else{   
                                                  if(x[2] <= 1024)  return 0;
                                                  else   return -1;
                                              }
                                          }
                                      }
                                  }
                              }
                          }
                      }
                  }
              }
          }
      }else{   
          if(x[5] <= 29)  return -1;
          else{   
              if(x[5] <= 31){  
                  if(x[0] >= 706)  return 1;
                  else{   
                      if(x[2] <= 640)  return 1;
                      else  return -1;
                  }
              }else{   
                  if(x[5] <= 36)  return -1;
                  else{   
                      if(x[5] <= 38){  
                          if(x[11] <= 0.3)  return -1;
                          else   return 1;
                      }else{   
                          if(x[5] <= 49){  
                              if(x[11] >= 0.4){  
                                  if(x[5] >= 48)  return -1;
                                  else{   
                                      if(x[0] <= 1585)  return 0;
                                      else   return -1;
                                  }
                              }else   
                                  if(x[5] <= 41)  return -1;
                                  else{   
                                      if(x[5] >= 44)  return -1;
                                      else{   
                                          if(x[2] >= 1344)  return -1;
                                          else   
                                              if(x[9] >= 3)  return -1;
                                              else{   
                                                  if(x[0] <= 624)  return -1;
                                                  else   return 0;
                                              }
                                          }
                                  }
                          }else{  
                              if(x[10] >= 7)  return -1;
                              else{   
                                  if(x[2] >= 1344){  
                                      if(x[5] <= 50)  return -1;
                                      else{   
                                          if(x[0] >= 2098)  return 0;
                                          else{   
                                              if(x[11] >= 0.45)  return 0;
                                              else{    
                                                  if(x[7] >= 11)  return 0;
                                                  else{   
                                                      if(x[8] <= 10)  return -1;
                                                      else   return 0;
                                                  }
                                              }
                                          }
                                      }
                                  }else{   
                                      if(x[4] >= 1)  return 0;
                                      else{   
                                          if(x[1] >= 4032)  return 0;
                                          else{   
                                              if(x[0] >= 598)  return 0;
                                              else {  
                                                  if(x[10] >= 1)  return 0;
                                                  else{    
                                                      if(x[11] <= 0.05)  return -1;
                                                      else{   
                                                          if(x[1] >= 3328)  return 0;
                                                          else{   
                                                              if(x[0] <= 92)  return -1;
                                                              else{   
                                                                  if(x[2] <= 256)  return 0;
                                                                  else   return -1;
                                                              }
                                                          }
                                                      }
                                                  }
                                              }
                                          }
                                      }
                                  }
                              }
                          }
                      }
                  }
              }
          }
      }
  }
}

int TreeDepth3(float *x){
  if(x[3] > 0){
      if(x[5] > 25) return 0;
      else{ 
          if(x[7] > 0.4) return 0;
          else{ 
              if(x[7] > 0.2) return 0;
              else{ 
                  if(x[0] > 795) return -1;
                  else {
                      if(x[0] > 440) return 0;
                      else{  
                          if(x[2] <= 640) return 0;
                          else return -1;
                      }    
                  }        
              }
          }
      }
  }else{
      if(x[5] <= 29 ){
          if(x[5] > 23) return -1;
          else{  
              if(x[7] <= 0.85) return -1;
              else{  
                  if(x[2] > 1280) return -1;
                  else{  
                      if(x[0] <= 430) return -1;
                      else  return 0;
                  }
              }
          }
      }else{  
          if(x[5] <= 43){ 
              if(x[7] > 0.45){ 
                  if(x[7] > 0.55){ 
                      if(x[0] > 764) return 0;
                      else{  
                          if(x[2] <= 768) return 0;
                          else return -1;
                      }
                  }else  
                      if(x[5] > 38) return 0;
                      else{  
                          if(x[0] > 752) return 0;
                          else { 
                              if(x[2] <= 704) return 0;
                              else  return -1;
                          }
                      }
              }else{  
                  if(x[5] <= 31){ 
                      if(x[0] > 690) return 0;
                      else { 
                          if(x[2] > 640) return -1;
                          else{  
                              if(x[0] > 363) return 0;
                              else{  
                                  if(x[2] <= 320) return 0;
                                  else  return -1;
                              }
                          }
                      }
                  }else{  
                      if(x[7] <= 0.35) return -1;
                      else{  
                          if(x[5] <= 38){ 
                              if(x[5] <= 37) return -1;
                              else{  
                                  if(x[0] <= 2021) return 0;
                                  else return -1;
                              }
                          }else{  
                              if(x[7] <= 0.4) return -1;
                              else{  
                                  if(x[0] > 1858) return -1;
                                  else{  
                                      if(x[5] <= 42) return 0;
                                      else  return -1;
                                  }
                              }
                          }
                      }
                  }
              }
          }else{  
              if(x[7] <= 0.6){ 
                  if(x[7] <= 0.3) return -1;
                  else{  
                      if(x[5] > 48) return -1;
                      else{  
                          if(x[7] <= 0.5) return -1;
                          else{  
                              if(x[0] <= 105) return 0;
                              else  return -1;
                          }
                      }
                  }
              }else{  
                  if(x[5] > 50){ 
                      if(x[7] <= 0.8){ 
                          if(x[2] <= 64) return 0;
                          else  return -1;
                      }else{  
                          if(x[4] <= 0) return 0;
                          else{ 
                              if(x[6] <= 2) return 0;
                              else{  
                                  if(x[1] <= 3584) return -1;
                                  else  return 0;
                              }
                          }
                      }
                  }else{  
                      if(x[2] > 1600){ 
                          if(x[0] <= 1663) return -1;
                          else{  
                              if(x[5] <= 45) return -1;
                              else{  
                                  if(x[7] <= 0.75) return -1;
                                  else  return 0;
                              }
                          }
                      }else{  
                          if(x[7] > 0.8){ 
                              if(x[5] > 49) return 0;
                              else{  
                                  if(x[5] <= 48) return 0;
                                  else  return -1;
                              }
                          }else{  
                              if(x[5] <= 48){ 
                                  if(x[2] <= 1536) return 0;
                                  else  return -1;
                              }else{  
                                  if(x[5] <= 49) return -1;
                                  else{  
                                      if(x[7] <= 0.65) return -1;
                                      else  return 0;
                                  }
                              }
                          }
                      }
                  }
              }
          }
      }
  }

}


//! \}

//! \}