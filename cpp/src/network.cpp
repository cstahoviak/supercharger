/*************************************************/
/********* DO NOT MODIFY THIS FILE ************/
/*************************************************/
#include "network.h"

namespace supercharger
{
  std::ostream& operator<<(std::ostream& stream, const Charger& charger) {
    return stream << "Charger(name: '" << charger.name << "' , lat: " << 
      charger.lat << ", lon: " << charger.lon << ", rate: " << 
      charger.rate << " km/hr)";
  }

  std::array<Charger, 303> network = 
  {{
  {"Albany_NY", 42.710356, -73.819109, 131.0},{"Edison_NJ", 40.544595, -74.334113, 159.0},{"Dayton_OH", 39.858702, -84.277027, 133.0},{"Boise_ID", 43.592251, -116.27942, 143.0},{"Lumberton_NC", 34.667629, -79.002343, 105.0},{"Albuquerque_NM", 35.108486, -106.612804, 175.0},{"Newark_DE", 39.662265, -75.692027, 120.0},
  {"West_Lebanon_NH", 43.623536, -72.3258949, 153.0},{"West_Wendover_NV", 40.738399, -114.058998, 106.0},{"Salina_KS", 38.877342, -97.618699, 177.0},{"Glen_Allen_VA", 37.66976, -77.461414, 128.0},{"Beaver_UT", 38.249149, -112.652524, 109.0},{"Pleasant_Prairie_WI", 42.518715, -87.950428, 144.0},{"Independence_MO", 39.040814, -94.369265, 107.0},{"Redondo_Beach_CA", 33.894227, -118.367407, 114.0},{"Yuma_AZ", 32.726686, -114.619093, 116.0},{"Milford_CT", 41.245823, -73.009059, 130.0},{"Liverpool_NY", 43.102424, -76.187446, 138.0},
  {"Columbia_MO", 38.957778, -92.252761, 109.0},{"Harrisburg_PA", 40.277134, -76.823255, 141.0},{"Turkey_Lake_FL", 28.514873, -81.500189, 133.0},{"Lake_City_FL", 30.181405, -82.679605, 86.0},{"Fremont_CA", 37.394181, -122.149858, 151.0},{"Bozeman_MT", 45.70007, -111.06329, 105.0},{"Peru_IL", 41.348503, -89.126115, 158.0},{"Pendleton_OR", 45.64655, -118.68198, 82.0},{"Ann_Arbor_MI", 42.241125, -83.766522, 103.0},{"Needles_CA", 34.850835, -114.624329, 102.0},{"Lebec_CA", 34.98737, -118.946272, 180.0},
  {"Atlanta_GA", 33.79382, -84.39713, 145.0},{"Winnemucca_NV", 40.958869, -117.746501, 114.0},{"Queens_NY", 40.66179, -73.79282, 149.0},{"Country_Club_Hills_IL", 41.585206, -87.721114, 88.0},{"Flagstaff_AZ", 35.174151, -111.663194, 144.0},{"Norfolk_VA", 36.860525, -76.207467, 128.0},{"Uvalde_TX", 29.112378, -99.75208, 140.0},{"Tannersville_PA", 41.045431, -75.312237, 174.0},{"Centralia_WA", 46.729872, -122.977392, 159.0},{"Southampton_NY", 40.891909, -72.426995, 104.0},{"Seaside_CA", 36.61697, -121.843973, 110.0},
  {"Dublin_CA", 37.703163, -121.925304, 175.0},{"Lexington_KY", 38.017955, -84.420664, 122.0},{"Napa_CA", 38.235578, -122.263886, 155.0},{"Augusta_ME", 44.347885, -69.786042, 129.0},{"Nephi_UT", 39.678111, -111.841003, 141.0},{"Green_River_UT", 38.993577, -110.140513, 146.0},{"Plattsburgh_NY", 44.704537, -73.491829, 169.0},{"Hooksett_NH", 43.109066, -71.477768, 121.0},{"Cisco_TX", 32.374263, -99.007197, 181.0},{"Cadillac_MI", 44.28254, -85.40306, 111.0},{"Cranbury_NJ", 40.32244, -74.4869, 103.0},
  {"Charlotte_NC", 35.34075, -80.76579, 164.0},{"Indio_CA", 33.741291, -116.215029, 167.0},{"Alexandria_LA", 31.312424, -92.446436, 160.0},{"Maumee_OH", 41.57833, -83.664593, 134.0},{"Ellensburg_WA", 46.976918, -120.54162, 139.0},{"Savannah_GA", 32.135885, -81.212853, 100.0},{"Holbrook_AZ", 34.922962, -110.145558, 132.0},{"Fresno_CA", 36.835455, -119.91058, 113.0},{"Newburgh_NY", 41.499616, -74.071324, 194.0},{"Temecula_CA", 33.52421, -117.152568, 98.0},{"South_Burlington_VT", 44.46286, -73.179308, 117.0},
  {"Folsom_CA", 38.642291, -121.18813, 148.0},{"Gardnerville_NV", 38.696385, -119.548525, 100.0},{"London_KY", 37.14916, -84.11385, 190.0},{"Casa_Grande_AZ", 32.878773, -111.681694, 158.0},{"San_Marcos_TX", 29.827707, -97.979685, 126.0},{"Corsicana_TX", 32.068583, -96.448248, 125.0},{"El_Centro_CA", 32.760837, -115.532486, 128.0},{"Onalaska_WI", 43.879042, -91.188428, 130.0},{"Darien_CT", 41.080103, -73.46135, 150.0},{"Sandy_OR", 45.402786, -122.294371, 119.0},{"Superior_MT", 47.192149, -114.888901, 125.0},
  {"Manteca_CA", 37.782622, -121.228683, 128.0},{"Ocala_FL", 29.140981, -82.193938, 127.0},{"Santa_Rosa_NM", 34.947013, -104.647997, 164.0},{"Santee_SC", 33.485858, -80.475763, 188.0},{"South_Salt_Lake_City_UT", 40.720352, -111.888712, 167.0},{"Sparks_NV", 39.541124, -119.442336, 188.0},{"Allentown_PA", 40.588118, -75.560089, 183.0},{"Knoxville_TN", 35.901319, -84.149634, 126.0},{"Moab_UT", 38.573122, -109.552368, 121.0},{"Denver_CO", 39.77512, -104.794648, 160.0},{"Brandon_FL", 27.940665, -82.323525, 146.0},
  {"Rapid_City_SD", 44.105601, -103.212569, 128.0},{"West_Yellowstone_MT", 44.656089, -111.099022, 135.0},{"Burlington_WA", 48.509743, -122.338681, 121.0},{"Cheyenne_WY", 41.161085, -104.804955, 179.0},{"Dedham_MA", 42.236461, -71.178325, 146.0},{"West_Springfield_MA", 42.130914, -72.621435, 139.0},{"Port_St._Lucie_FL", 27.31293, -80.406743, 129.0},{"Somerset_PA", 40.017517, -79.07712, 133.0},{"San_Rafael_CA", 37.963357, -122.515699, 89.0},{"St._Joseph_MI", 42.056357, -86.456352, 124.0},{"San_Mateo_CA", 37.5447, -122.29011, 118.0},
  {"Vienna_VA", 38.931919, -77.239564, 84.0},{"Brentwood_TN", 35.9696, -86.804159, 183.0},{"Ukiah_CA", 39.1481, -123.208604, 153.0},{"Aurora_IL", 41.760671, -88.309184, 105.0},{"San_Diego_CA", 32.902166, -117.193699, 102.0},{"Hawthorne_CA", 33.921063, -118.330074, 158.0},{"Grove_City_OH", 39.877253, -83.063448, 155.0},{"Gallup_NM", 35.505278, -108.828094, 161.0},{"Butte_MT", 45.981226, -112.507161, 84.0},{"Grants_Pass_OR", 42.460931, -123.324124, 118.0},{"Queensbury_NY", 43.328388, -73.679992, 118.0},{"Colorado_Springs_CO", 38.837573, -104.824889, 158.0},
  {"Highland_Park_IL", 42.17434, -87.816626, 138.0},{"Hays_KS", 38.900543, -99.319142, 156.0},{"St._Charles_MO", 38.78216, -90.5329, 115.0},{"Paramus_NJ", 40.957892, -74.073976, 114.0},{"Lone_Tree_CO", 39.563776, -104.875651, 188.0},{"Cleveland_OH", 41.519427, -81.493146, 146.0},{"Bellmead_TX", 31.582287, -97.109152, 132.0},{"Seabrook_NH", 42.895248, -70.869299, 108.0},{"Missoula_MT", 46.914375, -114.031924, 114.0},{"Watertown_NY", 43.979585, -75.954114, 166.0},{"Atascadero_CA", 35.486585, -120.666378, 94.0},{"Murdo_SD", 43.886915, -100.716887, 121.0},
  {"Burbank_CA", 34.174754, -118.300803, 179.0},{"Sunnyvale_CA", 37.405893, -121.987945, 150.0},{"Laurel_MD", 39.095382, -76.858319, 115.0},{"Oakdale_MN", 44.964892, -92.961249, 130.0},{"Buffalo_NY", 42.968675, -78.69568, 146.0},{"Culver_City_CA", 33.986765, -118.390162, 120.0},{"Fountain_Valley_CA", 33.70275, -117.934297, 125.0},{"Macon_GA", 32.833485, -83.625813, 160.0},{"Baxter_MN", 46.378836, -94.256378, 142.0},{"Madison_WI", 43.12669, -89.306829, 151.0},{"Angola_IN", 41.699048, -85.000326, 129.0},{"Effingham_IL", 39.137114, -88.563468, 131.0},
  {"Quartzsite_AZ", 33.660784, -114.241801, 123.0},{"Gilroy_CA", 37.02445, -121.56535, 155.0},{"Kennewick_WA", 46.198035, -119.162687, 157.0},{"Hamilton_Township_NJ", 40.195539, -74.641375, 110.0},{"Duluth_MN", 46.784467, -92.10232, 184.0},{"Terre_Haute_IN", 39.443345, -87.331737, 146.0},{"Egg_Harbor_Township_NJ", 39.393663, -74.562619, 79.0},{"Las_Vegas_NV", 36.165906, -115.138655, 84.0},{"Mammoth_Lakes_CA", 37.644519, -118.965499, 97.0},{"Strasburg_VA", 39.00496, -78.337848, 176.0},{"Wickenburg_AZ", 33.970281, -112.731503, 164.0},{"Limon_CO", 39.268975, -103.708626, 126.0},
  {"East_Greenwich_RI", 41.660517, -71.497242, 107.0},{"Riviera_Beach_FL", 26.77825, -80.109586, 113.0},{"Erie_PA", 42.049602, -80.086345, 144.0},{"Kingman_AZ", 35.191331, -114.065592, 98.0},{"Okeechobee_FL", 27.60089, -80.82286, 135.0},{"Big_Timber_MT", 45.83626, -109.94341, 166.0},{"Tucumcari_NM", 35.15396, -103.7226, 147.0},{"Baton_Rouge_LA", 30.423892, -91.154637, 173.0},{"The_Dalles_OR", 45.611941, -121.208249, 178.0},{"Greenwich_CT", 41.041538, -73.671661, 138.0},{"Dallas_TX", 32.832466, -96.837638, 101.0},{"Perry_OK", 36.289315, -97.325935, 144.0},
  {"Syosset_NY", 40.7999, -73.51524, 106.0},{"Cranberry_PA", 40.683508, -80.108327, 148.0},{"Greenville_SC", 34.729509, -82.366353, 96.0},{"Tonopah_NV", 38.069801, -117.232243, 119.0},{"Mountville_SC", 34.39359, -82.028798, 132.0},{"Pearl_MS", 32.274159, -90.151048, 141.0},{"Louisville_KY", 38.211962, -85.67319, 177.0},{"Buellton_CA", 34.614555, -120.188432, 155.0},{"Sheboygan_WI", 43.749753, -87.746971, 116.0},{"Bethesda_MD", 39.023876, -77.144352, 106.0},{"Victoria_TX", 28.766853, -96.978988, 165.0},{"Grand_Rapids_MI", 42.914231, -85.533057, 125.0},{"Tifton_GA", 31.448847, -83.53221, 185.0},
  {"Richfield_UT", 38.78799, -112.085173, 176.0},{"Columbus_TX", 29.690066, -96.537727, 142.0},{"Indianapolis_IN", 39.702238, -86.07959, 91.0},{"Triadelphia_WV", 40.06076, -80.602742, 115.0},{"Normal_IL", 40.508562, -88.984738, 162.0},{"Burlingame_CA", 37.593182, -122.367483, 130.0},{"Mountain_View_CA", 37.415328, -122.076575, 133.0},{"South_Hill_VA", 36.748516, -78.103517, 149.0},{"Chicago_IL", 41.890872, -87.654214, 144.0},{"Brooklyn_NY", 40.68331, -74.006508, 115.0},{"Buttonwillow_CA", 35.400105, -119.397796, 166.0},{"Beatty_NV", 36.913695, -116.754463, 127.0},
  {"Asheville_NC", 35.531428, -82.604495, 163.0},{"Corning_CA", 39.92646, -122.1984, 134.0},{"Shreveport_LA", 32.478594, -93.75437, 95.0},{"Farmington_NM", 36.766315, -108.144266, 143.0},{"Billings_MT", 45.734046, -108.604932, 119.0},{"Matthews_NC", 35.140024, -80.719776, 110.0},{"Twin_Falls_ID", 42.597887, -114.455249, 146.0},{"Vacaville_CA", 38.366645, -121.958136, 123.0},{"St._Augustine_FL", 29.924286, -81.416018, 137.0},{"Lake_Charles_LA", 30.199071, -93.248782, 134.0},{"Tinton_Falls_NJ", 40.226408, -74.093572, 113.0},{"Stanfield_AZ", 32.949077, -111.991933, 92.0},
  {"Grand_Junction_CO", 39.090758, -108.604325, 107.0},{"Coeur_d'Alene_ID", 47.708479, -116.794283, 113.0},{"Lindale_TX", 32.470885, -95.450473, 135.0},{"Orlando_FL", 28.617982, -81.387995, 124.0},{"Binghamton_NY", 42.145542, -75.902081, 157.0},{"Hagerstown_MD", 39.605859, -77.733324, 121.0},{"DeFuniak_Springs_FL", 30.720702, -86.116677, 123.0},{"Slidell_LA", 30.266552, -89.760156, 124.0},{"Kingsland_GA", 30.790734, -81.663625, 130.0},{"Catoosa_OK", 36.167631, -95.766044, 132.0},{"Port_Huron_MI", 42.998817, -82.428935, 86.0},{"Marathon_FL", 24.72611, -81.047912, 154.0},
  {"Goodland_KS", 39.326258, -101.725107, 140.0},{"Cherry_Valley_IL", 42.243893, -88.978895, 101.0},{"Truckee_CA", 39.327438, -120.20741, 158.0},{"Monterey_CA", 36.612153, -121.897995, 165.0},{"Blue_Ash_OH", 39.224642, -84.383507, 127.0},{"Rocky_Mount_NC", 35.972904, -77.846845, 180.0},{"Inyokern_CA", 35.646451, -117.812644, 178.0},{"Sagamore_Beach_MA", 41.781195, -70.540289, 114.0},{"West_Hartford_CT", 41.722672, -72.759717, 106.0},{"Hinckley_MN", 46.009797, -92.93137, 169.0},{"Bowling_Green_KY", 36.955196, -86.438854, 145.0},{"Oxnard_CA", 34.238115, -119.178084, 104.0},
  {"Auburn_AL", 32.627837, -85.445105, 111.0},{"Costa_Mesa_CA", 33.673925, -117.882412, 119.0},{"Roseville_CA", 38.771208, -121.266149, 138.0},{"East_Brunswick_NJ", 40.415938, -74.444713, 153.0},{"Bellevue_WA", 47.62957, -122.148073, 145.0},{"St._George_UT", 37.126463, -113.601737, 183.0},{"Buckeye_AZ", 33.443011, -112.556876, 154.0},{"San_Juan_Capistrano_CA", 33.498538, -117.66309, 135.0},{"Oklahoma_City_OK", 35.461664, -97.65144, 87.0},{"Lima_OH", 40.726668, -84.071932, 159.0},{"Weatherford_OK", 35.53859, -98.66012, 116.0},{"Ritzville_WA", 47.116294, -118.368328, 118.0},
  {"Trinidad_CO", 37.134167, -104.519352, 121.0},{"Denton_TX", 33.231373, -97.166412, 154.0},{"Sweetwater_TX", 32.450591, -100.392455, 145.0},{"Champaign_IL", 40.146204, -88.259828, 144.0},{"Gillette_WY", 44.292984, -105.526325, 135.0},{"Barstow_CA", 34.849124, -117.085459, 127.0},{"Mobile_AL", 30.671556, -88.118644, 98.0},{"Glenwood_Springs_CO", 39.552676, -107.340171, 125.0},{"Miner_MO", 36.893583, -89.533986, 153.0},{"Eureka_CA", 40.778885, -124.188383, 135.0},{"Plantation_FL", 26.108605, -80.252444, 113.0},{"Idaho_Falls_ID", 43.485152, -112.05205, 142.0},
  {"Utica_NY", 43.113878, -75.206857, 133.0},{"Fort_Myers_FL", 26.485574, -81.787149, 106.0},{"Yucca_AZ", 34.879736, -114.131562, 131.0},{"Albert_Lea_MN", 43.68606, -93.357721, 92.0},{"Sheridan_WY", 44.804582, -106.956345, 95.0},{"Sulphur_Springs_TX", 33.137098, -95.603229, 151.0},{"Villa_Park_IL", 41.907415, -87.973023, 129.0},{"Mayer_AZ", 34.32753, -112.11846, 142.0},{"Gila_Bend_AZ", 32.943675, -112.734081, 131.0},{"Mishawaka_IN", 41.717337, -86.18863, 109.0},{"Tempe_AZ", 33.421676, -111.897331, 187.0},{"Silverthorne_CO", 39.631467, -106.070818, 163.0},
  {"Huntsville_TX", 30.716158, -95.565944, 154.0},{"Price_UT", 39.600831, -110.831666, 163.0},{"Lone_Pine_CA", 36.60059, -118.061916, 105.0},{"Amarillo_TX", 35.189016, -101.931467, 98.0},{"Woodburn_OR", 45.15313, -122.881254, 139.0},{"Primm_NV", 35.610678, -115.388014, 115.0},{"Lincoln_City_OR", 44.957751, -124.010966, 136.0},{"Blanding_UT", 37.625618, -109.473842, 148.0},{"Brattleboro_VT", 42.838443, -72.565798, 107.0},{"Springfield_OR", 44.082607, -123.037458, 92.0},{"Cabazon_CA", 33.931316, -116.820082, 169.0},{"Pocatello_ID", 42.899615, -112.435248, 96.0},
  {"Mt._Shasta_CA", 41.310222, -122.31731, 103.0},{"Decatur_GA", 33.793198, -84.285394, 128.0},{"Bend_OR", 44.03563, -121.308473, 186.0},{"Coalinga_CA", 36.254143, -120.23792, 159.0},{"Wytheville_VA", 36.945693, -81.054651, 142.0},{"Chattanooga_TN", 35.038644, -85.19593, 113.0},{"Port_Orange_FL", 29.108571, -81.034603, 165.0},{"Wichita_KS", 37.60878, -97.33314, 154.0},{"Macedonia_OH", 41.313663, -81.517018, 159.0},{"Tremonton_UT", 41.70995, -112.198576, 99.0},{"Plymouth_NC", 35.850587, -76.756116, 107.0},{"Petaluma_CA", 38.242676, -122.625023, 134.0},
  {"Lafayette_IN", 40.41621, -86.814089, 126.0},{"Detroit_OR", 44.73704, -122.151999, 99.0},{"Palo_Alto_CA", 37.394011, -122.150347, 124.0},{"Mojave_CA", 35.068595, -118.174576, 169.0},{"Eau_Claire_WI", 44.77083, -91.43711, 142.0},{"Mitchell_SD", 43.701129, -98.0445, 125.0},{"Lee_MA", 42.295745, -73.239226, 151.0},{"Houston_TX", 29.980687, -95.421547, 124.0},{"East_Liberty_OH", 40.303817, -83.550529, 145.0},{"Tallahassee_FL", 30.510908, -84.247841, 182.0},{"Lovelock_NV", 40.179476, -118.472135, 168.0},{"Ardmore_OK", 34.179106, -97.165632, 143.0},
  {"Baker_City_OR", 44.782882, -117.812306, 163.0},{"Woodbridge_VA", 38.64082, -77.29633, 97.0},{"Rocklin_CA", 38.80086, -121.210529, 125.0},{"Elko_NV", 40.836301, -115.790859, 108.0},{"Reno_NV", 39.489732, -119.794179, 142.0},{"Lusk_WY", 42.75625, -104.45267, 136.0},{"Shamrock_TX", 35.226765, -100.24836, 173.0},{"Tooele_UT", 40.684466, -112.269008, 126.0},{"Salisbury_MD", 38.4016, -75.56489, 108.0},{"Council_Bluffs_IA", 41.220921, -95.835579, 165.0},{"Topeka_KS", 39.04438, -95.760267, 122.0},{"Rancho_Cucamonga_CA", 34.113584, -117.529427, 108.0},
  {"Worthington_MN", 43.63385, -95.595647, 108.0},{"Mauston_WI", 43.795551, -90.059358, 138.0},{"Warsaw_NC", 34.994625, -78.13567, 135.0}
  }};
} // end namespace supercharger

/*************************************************/
/********* DO NOT MODIFY THIS FILE ************/
/*************************************************/
