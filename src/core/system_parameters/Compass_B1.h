/*!
 * \file BeiDou_B1.h
 * \brief  Defines system parameters for Galileo E1 signal and NAV data
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 * \author Mara Branzanti 2013-2014. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013-2014. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_COMPASS_B1_H_
#define GNSS_SDR_COMPASS_B1_H_

#include <complex>
#include <gnss_satellite.h>
#include <string>
#include <vector>
#include <utility> // std::pair
#include "MATH_CONSTANTS.h"

// Physical constants
const double COMPASS_PI = 3.1415926535898;
const double COMPASS_GM = 3.986004418e14;  //!< Earth's universal gravitational constant of CGCS2000[m^3/s^2] as defined in BeiDou ICD
const double COMPASS_OMEGA_EARTH_DOT = 7.2921150e-5;  //!< Value of the earth's rotation rate of CGCS2000 [rad/s]
const double COMPASS_C_m_s = 299792458.0;  //!< The speed of light, [m/s]
const double COMPASS_C_m_ms = 299792.4580; //!< The speed of light, [m/ms]
//const double GALILEO_F = -4.442807633e-10; //!< Constant, [s/(m)^(1/2)]

// carrier and code frequencies
const double Compass_B1_FREQ_HZ = 1.561098e9;             //!< BeiDou B1 carrier frequency [Hz]
const double Compass_B1_CODE_CHIP_RATE_HZ = 2.046e6;      //!< BeiDou B1 code rate [chips/s]
const double Compass_B1_CODE_PERIOD = 0.001;             //!< BeiDou B1 code period [s]
const double Compass_B1_CODE_LENGTH_CHIPS = 2046.0;    //!< BeiDou B1 code length [chips]
const int Compass_B1_NUMBER_OF_CODES = 37;
const double Compass_NH_SECONDARY_CODE_LENGTH = 20;  //!< Compass NH code length [chips]

//const double Galileo_E1_FREQ_HZ = 1.57542e9;             //!< Galileo E1 carrier frequency [Hz]
//const double Galileo_E1_CODE_CHIP_RATE_HZ = 1.023e6;    //!< Galileo E1 code rate [chips/s]
//const double Galileo_E1_CODE_PERIOD = 0.004;             //!< Galileo E1 code period [s]
//const double Galileo_E1_SUB_CARRIER_A_RATE_HZ = 1.023e6; //!< Galileo E1 sub-carrier 'a' rate [Hz]
//const double Galileo_E1_SUB_CARRIER_B_RATE_HZ = 6.138e6; //!< Galileo E1 sub-carrier 'b' rate [Hz]
//const double Galileo_E1_B_CODE_LENGTH_CHIPS = 4092.0;    //!< Galileo E1-B code length [chips]
//const double Galileo_E1_B_SYMBOL_RATE_BPS = 250.0;       //!< Galileo E1-B symbol rate [bits/second]
//const double Galileo_E1_C_SECONDARY_CODE_LENGTH = 25.0;  //!< Galileo E1-C secondary code length [chips]
//const int Galileo_E1_NUMBER_OF_CODES = 50;

const double COMPASS_STARTOFFSET_ms = 68.802; //[ms] Initial sign. travel time (this cannot go here)

// Galileo E1 primary codes
const std::string Compass_B1_PRIMARY_CODE[Compass_B1_NUMBER_OF_CODES] = {
		"65B6CDD4D7C12AA430F803869937C509C80C7A022ABCC43E4B4508E93484F4D03576E6B64D82FC3387B568C690824511A17044437E781759B5E8EDE953C76C640275D12B8DF4898E7676BDB9B972E571B3B7383776894CB192A941485E2FB73A28AF1C8A670DFEDFA637FD94D14965994785BB393E58CEE08C4BD50F3CD4ABB80043DBC8F3DE9CFB8D504FCB51F320E9C8D2B8AF3CB231B3A86CBFBD526BE2BA0CE33CBFC5DACE4F64A96CBCD80B852978850C6CD0AD77186B36712E7822045DEC3748512EDEAA115C25C52D354DA987EDA178E945B6F8C8CB53F89B10ECAA4D68352D7EC9FD682C28EF44E3C265C4399246E1BE67764E8B8D09B91E8C51D0D4",
		"9262381AB6974F65608285424B4FAA4B4C40DA553BBEBF5EAA0B1036365D88BAA4FE067F84B8D4D5513BC64688FF5B43028AEA008EF79C14C3605C2EDF128C9D31735C8AAE26A0EF711D39060BEBC42BE8F9411C0D1ABF31BA2E63BFBA2045FE393F815DFC942C973349DC9F8442F37A23F3C09A37E443205BE0B6E1D10620C58CF7CF488A57FBE3A1405887634286916478CE76833052AE57700498FEF16E5F1626AAFEBD276A7C52DEF13A4DD7B8A9F21EA544A49D4ECC3CE49BA78DDC9F372CB1ED772FFB677FA98A50021A03134464A794BC1F2E5E437537BE1A8F405135F4656702318A57B636ECE95C83EE2B539128B60D7986F1FAFCDA765A175AE428",
		"698842FD863C7D85C8BFC62022739DEA0E668A7EB33F82EEDAAC1C59B731368FEC3A761B6025C0A63A7C910684C1D46A5377BD2176B059B2782404CD19787CE1A8F01A5A3FCFB45FF2A8FB59D2A75486C55E7D89B0D346F1AE6DF2C44827BC9C31F7CFB63158C5B379F6CC1A2EC7380B91C8FD4BB33A05C030350716A7EF657B4AADC508B693486FB74853217A1A55AD322DF51A5CF16320A8FE590A28BC282D9B4461DE0159B865C9E53FF90739A669B75371D09E855226170DEEE37723D2824CF2BFE42F6981C8D35D9A958DA44E25A024E296B2620D06AA059D5A40962C89BA4D423C4DB1C87B39ED3F83232BDCE6909F9DD4F6FEAE42443391F85ADF7E54",
		"947D7F8E1E69E4F59CA1679116ED863AAF75A26B777F1C36E2FF9A6E7787699548584E29126B4A9F8FDF3AA682DE93FEFB8916B18A93BB61258628BCFA4D04DFE431B932773B3E07B3721A763E011CD0538DE3C36E37BA11A44C3A79B124402D3593E8C3D7BEB1215CA94458FB85DDB348D563A3715526B005DFDFED1C9BC7A42980C028A8F111A9BC4C56F276B63C33190768AC3311FBE7D73977C3439A8B14DDF5044E5F66D1690478D898A24EA90995F59B9A83895C5302F954410A5C7458FCD316ADAF20F2936E367FDE4677E09542655983E4C424A4459C8CFA277D12579D5950A373AC079DBE6DD4ECF349273C104408383142819E184762297C1DB368",
		"95FAAE6B34564E11A3A99F25C1850D26D7B8FC64E44F74ECF0C2B8F867F1D1DEB371DC3CB7DFAD0D3866059E8056E1D185C9E99DCB8AB2BC5C3FB598731AE6CF5161005C417C59BD83E9D2AA337BAAEEBD50CB74B6FCFB59A3D4AC0833E54159F6B8F2A45D74164CC751626924741601121CCB6D20F97FD4125005AE2FFCBE7C40DD03F0A058AB3B3B0F556F33CB129B8658811A9F1991B5B72B2B15ACC072462F01EFE267F6BFAC51D152F019A8ED618C0F542D0A4C58F4CD7E27B8ABBC4E8308CBA85B0F1617E85FD97429516ADC610BD5EACC1A3EBA1D89B640420D72428F07165D4B232593975C4D9B80EF60E427F01F27B563739D8721602735A6CC66B8",
		"6A4409C5475CFD3FA92A4B13E716CE5CC39A99665CC76737F7C8C83E9FE71A3DE7FD9B3AF9967C4A0ED270EA8095092310D63CEFD40ECEE6378BF0164F7C49C898F934314862C8F68BD28E8FCEEF63E46F8AB8BDED2064C5A290951F8CC53ECFD634764AE1A8D8DE83FA93617EDC4AB6093F78B038B49BBA14ED5EB158922A27ACB8A354A394E003FA6FD5D5525E9FA8433DD2AC52E582AD58D3CECC81A4A62107D7C3506C31528DC862EE1C2D060C8D885B896449EDD93A6FC0B0ECE413BA585ECF9D723F1F3983287408802810A9B7179DDDAEB0EA7F29D4456276018F2554C3F4DF18C4E62A6B8CBD86ED156CBB5CA0045508FB84187CAAEEB94F82143F1C",
		"959B5A127ED9A4A8AC6BA108F45F2FE1C98BABE700836EDA744DF05DE3EC7FCC4DBBB8B9DEB294E995884A5080F4FD5A5A59D656DBCCF0CB0251D2D1514F1E4B7C352E07CCED80530FCF209D3025076106E7815940CE2B0BA232899453550104C672343DBFC6BF97A1AF6BE5538864ED84AEA15EB492698D17B3F33EE325600A5A8A7306A272C59F9ADF958862945931A18F7B77341B8B212F2FBC201716CC1293BCD50969D2A41D04BB306A37517C7B8A71E7C0E83D19DD3E9FFB46C3C44035F5CD87E6A71BAEB693A2B6D494AD935C19B9C61FE5801DB3FABCF36C07F196B921859E313707F695E4C5885BE86A94E10809EC5637FFDA816F29F672907813CC",
		"6A74F3F9E21B08632ECB54057DFBDF3F4C8332A7AEA16A2CB58F6C6C5DE9CD349898A9784D20E0B85825570D80C40766FF1E230A5C2DEFDD98BCC3B2DE56B58A8E53231C8EAA2401CDC1F7944F403523B2511DAB16390CECA26387D1BC9D1EE14E51150610F18C33308597A7452273C042664DA9F2811096961CA5F93EFEC51CA1931B2FA281D751AA87B5A6FAF13A7D50D62F9A87648FE714D185565C4FF90B59895E25EB235F5562D7DF513A7AC4008B64D092B8D579AE96305E93D02FBD03204C8AACEB19E52C4E49E9FECAF30E299EABCBC74F352CFEEDC03BE104CECF4FD0BD3EA5CEF718EAD0F98F0096E9833FDC0F30F951C23BFF8DCA51EC194E05A4",
		"B5D4360E571ABF48F7442C5AC54336D8EECEB73DD156BED97BEE3A099CB9B7B25C4AD1170FA4AC180F345576E025497E201CFA7636BC673D5175F99A38803B240E4B5E02DC09DEAF8E31781F0236BD9275DDC84BEF0A0A61034055F8DD9A7565F1E38C274AAAC24A1BFE0454C50AF4E2B582425B3690800B41873C2B766BBB3DAAC610B941E5A12A098E092B3A26911EBE3BFD11FAE83BF394A80C314463E91C15BBEF83938FF523BB685D322785D8BBAEC84A1E5DCFBC38D748A337E319CA9EA6D956916B937E85663DD9C71EA4DCC3175E355C25F5AA5BCBC11E2C6A3B60DAFF92701F0D7E1E0685A317A61D57E7795D909C02295E6BB5F13226D383500608",
		"A29804BFF16213E2819D93D8A65EC9C455EB1BA75DB3A41CEDAF42F4388A81C5C1CC44CB32242683A61C2146D9F11EACCA950F88ADD6A7BBF453F8BFF5EECE8B9B64714C66CA57A962642287A4BD03F6335C57E714729631493CAD8FE2006B665C5CCDACA5C0C1244C4D1AA47A7B85317AC6D950026F6FA2AA1BFE87BDE91BC8A74FB4499F60A9B349202B2FCCE753828F85F8A3023D18E2553B9E4515AE440148BBDF79AF2AB2D6E7EFBC2D311F35AACD4753B59C3DDA35CC0AE493B1DBC449AFA633A32927C819946B2EE0F9A48DCD52B1FD43C07D897AF06E291FCEAA4A26BD84DA0820B8D6EA72E504DF06D6482F9A4D47A73FA572DDD832D5C14D605E28",
		"59727E58C1C9210229A0D0BACF62FE6517CD4B8CD53299AC9D084E9BB9E63FF0890834AFD6B932F0CD5B7606D5CF91859B6858A95591621D4F17A05C33843EF702E7379CF7234319E1D1E0D87DF1935B1EFB6B72A9BB6FF15D7F3CF41007920454948347680C280006F20A21D0FE4E40C8FDE48186B12942C1CE4F70CB005E766115BE09A3A41A3F5F282089D5BF80BED9D0C3CFDDFC296CAAB5C3D7C3E30273C5D91459135460CF7CD472EE7BF12B6A880A8721A625C6DFE7E391D74B2489FCCFE5613029B52EAEEEBCE4776E03D0AC96328B696D31DA3F2F5C0A5F017C379AF3ACFF365C8349277DE4D200A613BF9A9BFA6C7EB0DD2D6560DB326300E5C454",
		"A487432B599CB8727DBE710BFBFCE5B5B6DE639911720774A55BC8AC795060EA2D6A0C9DA4F7B8C978F8DDA6D3D0D6113396F339A9B280CE12B58C2DD0B146C94E2694F4BFD7C941A00B01F79157DB0D8828F538775F9311575EF449E9046EB550F0A4328EEA5C9223AD826305BCABF811E07A6944DE0A32F424978B7074FCA90238BB29BDC643F9542C255AD913E920F2FA5E79B21CB1ABD572ED1EA8C5A14A836871C94D6B09C3B149958FDE86240AAAAC6D6BBB29C8AAF2172B75365B2F267FC4C879A9FC5DF553D7013CA5D07E1C7473307C3B97F39DC0C51BFF66970944D4B8EDA9629E86C1FA64396F767144401B21F992776102B93CAFC1B226270968",
		"A50092CE73A3129642B689BF2C946EA9CE133D9682426FAEB766EA3A6926D8A1D6439E8801435F5BCF41E29ED158A43E4DD60C15E8AB89136B0C110959E6A4D9FB762D9A8990AEFB9090C92B9C2D6D3366F5DD8FAF94D25950C662386BC56FC193DBBE550420FBFFB855A452DA4D604A4B29D2A715725356E3AB4DC8431385716B6578F1B56FF96BD36F26C79C6EC7886DA5B7CF1E14DBF9B560B1C8479F5818719C9A6575FB6706E4E01FE765606062B356A2DC32ECCC0D3D90588C97BB15FD8BDC768F09CAB88E62380ACBB2CD42E83DC38333C56D6D240CEFD7474C98599C4EF7E041321712CB184476036A58875BFB7AD61F25501EA0058884AEFCF6DCB8",
		"5ABE356000A9A1B848355D890A07ADD3DA3158943ACA7C75B06C9AFC9130134282CFD98E4F0A8E1CF9F597EAD19B4CCCD8C9D967F72FF54900B8548765800BDE32EE19F7808E3FB098AB950E61B9A439B42FAE46F4484DC551825B2FD4E51057B3573ABBB8FC356DFCFE555A80E53CFD500A617A0D3FB738E51616D7347D112A8700D855B6A3B253120FA67DFDFB4ABBA8C0E479D3E8C8E15A9854116AFB8C7F594AB6D77E3C8A277D53A30B51CE818EB7027F95714D4DC39F2ECFD8D814E126DDD843A639C396E515957662CBB7373E218BB4516FB9A810511CF57340653E478A156212D5D4AB37C8B46B6E9054D820AB61A4A2BDA79B5B8E061AD4D82E851C",
		"A56166B7392CF82F4D74B792194E4C6ED0206A15668E759833E9A29FED3B76B32889FA0D682E66BF62AFAD50D1FAB8B5924633DEF8EDCB64356276407BB35C5DD62203C1040177151CB63B1C9F73C0BCDD4297A259A6020B512047A40B752F9CA31178CCE6925224DEABADDEADB112A6DD9BB8948119450FE648BB588FCA5B0771320807B74597CF72BFE620CD318C224A724DA2B516C16D2D6426FDFC49E64CCD21A08E7BDF7CB7B18A7D7D4B99F178B5281131D09D8D24CE718472FFC31B4B76DA5932A1C701D0AE43C836770A0DD52FAFAFE03AD3CA8A7FE56469461B8DAA6864233B263577C9A0CC65D86D52F79D036C1DFC71DC59A64BC155E9CA42A9CC",
		"5A8ECF5CA5EE54E4CFD4429F90EABCB05528F355C8AC716EF22B3EAE533EC44BFDAAEBCCFBBC12EEAF02B00DD1CA42893701C6827F0CD472AF8F6723F4AAF79C24440EDA4646D347DEB8EC15E016F2FE69F40B500F5125EC517149E1E4BD30792B3259F749A561804F81519CBB1B058B1B535463C70A3C1467E7ED9F5211FE118A2B602EB7B6850142E7C60E5554EF6EBB2B194F0669C5AB169A1F8BB710D35507142BA2F92E87FFD7E6924646B24903B43D26638075ED5766DE21A7EC28E67DA35B5478EDC54A4A73A8971C295490A0A8BDA2389066FBC76899ACE44524D45C995C83AFDFC599B694F0628313D1E043D76AC15317E1B8D8A922F2774374BFA4",
		"AEA68B96A09F44C379DA567E1D1A91279381EBDBC430E2CC7C465644BB3F439A1880D4661F831A161BD5D886CDB28FD73892F6EAA51EE950399F119BBF51DE0E31E1BA3DD4F16A78E6BA6467CF68B20145B51259D2289C7175F81E03F40860C045041E90F395FA48938C2B2A85F5D8A3AC8B9F228F0DA48216652C9E26D2D50BEDA1AA89DA2D7D27733837C5E70E26C6757AB516627E4A7155A978F26F798E96DF1C82186BA9C4FC4AA3EF68EE2D16EA02912E09D215FF0BB0317B5EBEDA12960F63C4162890E3C01B137158414D6A6F1F34673C37A97CB491384CDE9ED0CCE26FFCB54AA4F476BD63E77FBFE79850F098943BCDAE2D92141108FD279BEEF0A8",
		"5353B6E538CADDB32DC4F7CF29848AF73292C3CE00707C144415D0737B891C80BCE2EC546DCD902FAE767326CBADC843906C5D7A593D0B83643D3DEA5C64A6307D2019559C05E020A760854823CEFA57D3668C130CCC60917FD9D6BE0D0B9C71416039E515738EDAB6D3A36850B73D1B759601CA4D6287F2238FF4659DA677D48E8CAFA9C44F24E1783C3216EBA24F585E5028A00D9ED2B62A6E563B045F2DAF99ADE7883596ADF0873E08094B5A198A2037C443CF19F17EA5C5C1FCC3A5B44CBF426D5FA8D9909BA67894138A9EC4DFFD75DC29610F55167EA15D7EF93BF23C48E8A7D59AE9B95BE46794D037FAAB2A184FAE216991BDC84D7C0EF6BD2C3D94",
		"52D4670012F5775712CC0F7BFEEC01EB4A5F9DC1934014CE5628F2E56BFFA4CB47CB7E41C87977BD19CF4C1EC925BA6CEE2CA2561824025E1D84A0CED5334420C870A03BAA42879A97FB4D942EB44C693DBBA4A4D40721D9784140CF8FCA9D05824B23829FB929B72D2B85598F46F6A92F5FA9041CCEDE9634002E26AEC10E0CE7D16C71CCE69E73FF7F318BAEDF61F0C10FC116A196B8E44A7C0AEDEB05D4FD6B590C240D06C335D2978261F0BC5DE239CD0BF446DCF5D96A42B20562458E974B5AD3A908EF75E097979FE49D83F82BB4C56F669FF5CBAFB28B91C6D334A2E4D2A7AA3DCA602D510647DBBC2BD36831F81481AC3BA0A1D1745B4BEA67FDE844",
		"AD6AC0AE61FFC479184FDB4DD87FC2915E7DF8C32BC807155122822393E96F28134739478630A6FA2F7B396AC9E6529E7B33772407A07E047630E540E955EB2701E89456A35C16D19FC011B1D3208563EF61D76D8FDBBE45790579D830EAE293A2C7A76C2365E72569807451D5EEAA1E347C1AD904833AF832BD7539D9AF9A570BB4CCD5CF2AD54B3E1FB131CF4AECC3046A92A06C6AABFCA584EF34C661009A438F209606C12E144B243E8DC412BC0E3D99D6BD057D7417C8FC25512DEA7A4C1D5EE68038E65B8BE03AE34DE4F98DFDA88D580435210E9BEF78B3F2DFC9C53F1645286E2DA394ADD6B7C6D1D1DF374AA80FF311A357242AFFD5D5904325B1E0",
		"52B59379587A9DEE1D0E3156CB36232C546CCA42778C0EF8D2A7BA40EFE20AD9B9011AC4A1144E59B42103D0C987A6E731BC9D9D0862402943EAC787F766BCA4E5248E6027D35E741BDDBFA32DEAE1E6860CEE892235F18B79A76553EF7ADD58B281E51B7D0B806C4BD58CD5F8BA8445B9EDC33788A5C8CF31E3D8B66218D07AFD861C87CECCF0D75EAFF16CFF802A5AE6D83B7B0A94A270D2789DD850D36AA9D7E436CF0322D88487FDE0FBDE45CCF83FB3B819A4ADB4F099A36EFB0A3D8021B65CFC14A0E2CCBE5BEC5D195844B716A6A943B5604B6C01C18122E8D9B776D2F4346947DE424853BECFC8672CD918F700024A4F6F2CE6D73A129AAD51499D30",
		"AD5A3A92C4B831259FAEC45B4292D3F2D1645302D9AE0A0E1365267151E7B8216C220B0532863A08798C1E8DC9B75CDB94FB68C18F835F3FD907D6E4787F17651742837B6594FA26D9D368AA528FD3A432BA727B74C2D66C79F66B1600B2C2BD3AA2C420D23CB3C8DAFF7097EE1093687F252FC0CEB6B1D4B04C8E71BFC3756C069F74AECE3FE2196EF7D14267E5491617816F96B9EBA6B6E986A4AE1B8A5FB01DD1BDE381D323CCE1910FC0D36E74833EA68F4BF445D483310CCB2E19D67D1763DDF15EECE0872486070233061A2A6321BB4E6DCAFE5D4CD6FDEA65DA882F24050CC9D327B2A62C8AF3CF3C525A0F29D40496E0091107A9D8F13D33D87F8B58",
		"A8B9CC020861EF5385F9B4AD40B8BD5670B493E588F141A434B2DC1CFAE5A2B5F4269C308950845CC5312466C793476AC1910A5BA17ACE25DF7965099A0E564CE4A35F850DECF49024D54717FA826AFAFEC1B086B10599516B9A47C5FF0C651349A8770ED8BF67FEFC6CB3EDFA32F66AC7AD3C1BC9BCC112485A4592EB4F326A48D6A5E9F88B976D6E3439B0F2FA9C64080513CCD25FE338D5E00BA9D2126BDD14CF2CA889E87FE91C05C6CA01B4074A657A10D7F501ED948E2CB4B8395AF9F9DF013FCCA84B762CDCAF5E841D3999BE39F6AA03CC430653A1937E3E36ED8F8006C082EBE6D22696EB66420F973F5C9F19F885F8E6E9E270F595E954F0A9A7E8",
		"A93E1DE7225E45B7BAF14C1997D0364A0879CDEA1BC1297E268FFE8AEA931AFE0F0F0E252CE463CE72881B5EC51B3545BFD1F577E063C7F8A6C0F82D1359B45C51F3E6EB3BAB932A144E8FCBF7F8DCC4101C983169CED8196C02D1B47DCD64678A836D695275C093679495DC25C33DD89D6494D5981098765FD59FD1D8284BB2218B6631F0222DFFE9773A2DB787B2CC975AFA7A7E57896AB5F2577F3D48928FE63BC704B178112C49AC4CA2BA5243227C80DF607CC4E93341ABC74198BAC3222B19813A087D9357ED4055730A24A54A7046194C32B998EA6DB9B2861CE2DF589C8F8F03B65BB29C09460D638B169F84F9A3AA75B4D8FE69CCB2AC482A787238",
		"5680BA495154F699B072982FB143F5301C5BA8E8A3493AA521858E4C1285D11D5B83492362ADB289443C6E2AC5D8DDB72ACE2005FFE7BBA2CD74BDA32F3F1B5B986BD28632B502611C75D3EE0A6C15CEC2C6EBF8321247856D46E8A3C2ED1BF1AA0FE987EEA90E01233F64D47F6B616F86472708805D7C185968C4CEAF46DFE9CDEEC695F3EE66C72817BA97D6123FFF523FA9CCB3AB9A725A0AB2A6102C46E8CEEDEBB6BABFFC0DD01FF04E8EFCA2CE78D402293F6568FDE3155015D71537F97D1DB4133874BD3C9AED29DA735ED09C6C0E2E2E986D5DDE304A90B2101FB883586D0D5051980B60D9B6100E711AC0FFA9B8D8C82C2F7B92473C32320EA02B9C",
		"A95FE99E68D1AF0EB5337234A20A148D164A9A69FF0D3348A200B62F6E8EB4ECF1C56AA045895A2ADF665490C5B929CE6041CABCF025858FF8AE9F64310C4CD87CA7C8B0B63A4AC498687DFCF4A6714BABABD21C9FFC084B6DE4F4281D7D243ABA49ABF0B0C76948016A9C50523F4F340BD6FEE60C7B8E2F5A36694114F195C43BDC16C7F208435B48A7FACAE6D8F966B08D0017D55593FE2DF6C04A869E2CDB5A86FDEFBF5C0A9D1CC62E3894ABD2387AFE6C8D9EB5A81AB24A1BBFF0C2CD94D61FAE87A0702A09213B978ECFE3EA77622A359FCD073F441EB301A816610B6EBA1C4C79A279D79EB1CE1EB88C1CEF4201B56196E054B96F82FB7D0F1CCC074C",
		"56B04075F41303C5379387392BAEE45393420329512F37BE63C22A1ED08B061424E67B61D61B2E7B12CB49CDC589D3F2C5063FE077C49A9962438E07BE15E7198EC1C5ABF47DEE965A66AAF58BC343091F1D4EEEC90B2FAC6DB5FA6DF2B53BDF326A8ACB1FF05AEC9040601244955819CD1E12114A68F734DB993F86C92A30D2C0C57EEEF2FB519578FFDAE47EBD9A2A41D454FA662A97381608F93CCDC719C290B376C33DADF1D57AAAC10399806A437BEB5BDFCE5DC8691AE5BE6AE32930A2039EA3CDEC726193FCD0C8A491BD7702E538384767B20E0909CFC925155E52984B24ECED5B8939E185F219E3F29FF89CD5B3BD39866958116018DA9195FA1124",
		"54CB2094BA0BDCC7EEEFEDA8A34E2D9AA96AE5FFDF81B7A61EDC78BD2A2545E4AB6D36175EAAE9F7C72BB0FEC30472D1172F5EE71C40252BFB62D45CF06CCC621D324583735F197255946EE41B5E949286CF067BB72A24F96623190984CE98D68EE74A1CB493B40142CB1D9EF081D86044790A3D5A7FBB066A3F472A635CE96D42A66311EE407439E2733FFEBB2BDB52BC7067CC11B711ADCA3579B6566E31B6A08AA294EF4778208431ABC31F254C425E26352A61C8E746545F7DE3E5C565F89B3828738834E00C502BB038C1F70BFA9207A259641FB1488220A3267B09E186BB9B9D9C88467D7A8EC6E60C5B74645E79783F997364D1B590C65F990CBABF04",
		"AB75873AC9016FE9E46C399E85DDEEE0BD4880FD6709A47D19D6087BD2338E07FFE1711110E338B0F19FC58AC3C79A2382308B9503C4597190D691D2CC0A6365D4AA71EE7A4188395DAF32C1E6CA5D98541575B2ECF6BB656767201E3BEEE740AE6BCEF2084F7A930660EC96AA2984D75F5AB9E042325F686C821C3514327D36AEC3C3B5ED8C3F012313BF44DABE56617915347ADC4B02B525CD9C6F7B0AE5D1885C8E26E48095011D82172F2B8BADAE5A72E86322696688F6E1EAB7AA6A9123CD3C1D5AB83DCE672786CC91B88D7E2C8E4F953BCECB747CDFD3811277F4865D7F791FCF6F85C4865E36FB61A1783B2529634D24EB93544E1B48C1E32862E6A0",
		"54AAD4EDF084367EE12DD38596940F5DB759B27C3B4DAD909A533018AE38EBF655A7529237C7D0136AC5FF30C3A66E5AC8BF612C0C06675CA50CB315D23934E630666BD8FECEC09CD9B29CD31800391D3D784C564118F4AB67C53C95E47ED88BBE2D8C8556211DDA24351412877DAA8CD2CB600ECE14AD5F6FDCB1BAAF85371B58F113E7EC6A1A9D43A3FF19EA7490F89BA79DA1BAB50B395231EE83EDB88FE21C37987FE1636391D15BC95931DCDD58585886C783B9A66FA7BEA11D8DBD6B4E663E07CE203959529C5072C5043044C7806B8E8A9BA116E6F12A1008718A35B09D085EE69C641878364EF5D75C7E1498816EF47A27E896B3DE8F8EDE3A0ECA70",
		"AB457D066C469AB5638D26881F30FF8332512B3C956FA9665B91AC29103D590E80844353A455A442A768E26DC39694666DF894708BE7784A3FE1A2765D209F27C20066C3BC8964CE1BBC4BDA67650B5F89CED0A417EFD34C679432D00BB6C76E360EADBEF9162E7EB51FE85091D7BDA114038CF98807D444EE73E77D725E920DA3E87BCEEC99085373FBDF377211F3B46AFEC94C09CA0FFF69CFD7F5A6E1BAFBD6021353639298D9B73726623CF76523594DB195D351C61C0F1104C89E569678B3BF0A846C3B12C841BB2DEF5A6ED9B207798352311427ABE656D88572B56C466C30FE726594F6070272F28C22FD0346556828D541D577CD3C6C2940B338DC18",
		"AAF256DFE33EC50DDB64C12A52B565FCC585DEF2F439CCA70BEB2AEDC245364C04C8E304B557DF224626FAB2C14FE80CFC7074B942DD50ACE96F0CF6455D817561FAC8804C06EF836D34FA1DEBB0EBA6BAC85D05343DFA2D60FFB66FB92FE6346D40D4958285DDFE9D98CAA775D84F650593112E139E060C7B0DC676275504EEC79E006DE5258593A450BCD99FC378C9E64ADDCC704368E745DFC0B994501C837AA8658ADC10FBC4482B9D47906DE9C6438827D4ABAC622F3966994E0B8AABF83924A3AC180B2B1C1669C766AF9042D8C7FF26743031EAC513F94DAA5DFBD685E53612273F0C508CBC16B40DBD51F83EC93862A9B9A24857226F84FFF2B33370",
		"552D0508DABB9C9ADE252B3141FC8441CF94EC73A87DC54A886E128EBE4E53BDAE8EC08792733781DD7CC008C12E1C75B6FF9E004D1F6E81DCB52E315B6ED6F68536D2B6C889A726E929540F157A8F23D3A564E199D3B5E3605DAAE466BFD9FF7D0696E2DCEBBAB7BFCD3223588C613E8802C8C09FB8F43B78536BF99CE24EC331ACD03FE4C3A00FC4E0FC84AF09BE5004F8741716BD616B3223B25502E276B0EEC373D3D9F30D5484F243318A3A993041A249700A7CA2C86839D2E42C5D51959226B938800FBC29ADBF7932132D7833C9DB3DC5655B885F3D00DCB05B8565680747530ECCED8C72D46EBABB4057D7836135DBF775D98AAAE7A8CBC2E0DF1FA0",
		"AAC2ACE3467930515C85DE3CC858749F4A9C7533065FC1BC49AC8EBF004BE1457BADD14601E143D010D1DD55C11EE64913B86B5CCAFE719746583F52D4777D377750DFAD8ACE03742B2783066A1FBD616713F813CF249204600CA4A18977C61AF525B7D973DC89132EE7CE614E2676134ECA2437D9AB8D20F9FC3D3E4139EBD5CAB5B816E430B2C1F4B8DCAA376CDD1CF5A120FAA5C265AD09DD8B2349BB43A924F6F8FF5B02F61CE29EAC0A8711214B40B77E225A94C2BBC09677313FB6ACA347A7B472CC0DF7B3705426184D73E5464EC9301DCFEEB9122A7C143D58BA3C9EF67FF39A351D620DE052BDE03ED4C05DB533075813E46BD4054B6C5C69E909C8",
		"AA93A2A6A9B12FB4D4A6FF07676F473BDBB6897110F5D6918F6462484658985EFA028781DC3AE6C6EBC8B57CC1EDF48723E04B72529B12DBB7016BBF670879F14CAEE6DBC197366DE112082AE8EE4629017F1728C20F2A7F611993F3D99FA6695D8A120C60377425FB66C32B02243D8993217B1D87F510557EEE30E6EB8CDA98DDC9709BE70FEB3705807C3ECE9C3363C19D27A1DB417273DDDB578C2F86A2D7C6155F61D234E0751D41FFDDBE9478DC45F6943949DD2306CA8745B063F2A54EC4228C11B0069242DA12059B6A570DE5D5930AA7CF8F4D6B60F3FE84577802B3C3A5D15D2B2E358E049EA7D6BA5B88F8312EA94AED2E0F516C2655B8C4074604",
		"557C0B4D3573837F56060A0AEECBB7E55EBE1031BED7D2674EA6FE79F85D2AA62F2196404FA892972665A821C1DD0EBB86A7BE2ED57A0DCD2DEC7ADCE811D230BEC8EBC083D0923F231CDF23978B746BB5C98BDA94F80D9861489DB63657B98CD5A93337CF0047816A4C3F69148E2AA455E997EAC1E6694EFF41662136577F8E26D018B2E7FCF9F935D85C1056F9502F30C4734C683E76B5E6256EFA64DF97CE0C20D44D50C51B3D7B2D10E6B3BFC0A744E3A36B193543756228E0657019587811A3815BFC04D9D807F95AB1340990905281077F653A7C26778F360954475B45329D71C9D2DEDBF130A2A08DC4D89F26E52875E58B13EE2F8EC5F2264D31506C",
		"AAA3589A0CF6DAE85347E011FD82565854AF22B0E293DB8ACD23C61A84564F578567B5C3688C7A34BD3F929BC1BCFAC2CC285497DAB833E01836581BF62285B35A04F1F6075FDA9AA7017131694110EEDCA4B23E3916425661EA813DE9C78647C5EF7140916E20C84819C7ED39DA04FFD8784E044DC09B79FC1FCBAE8DE035A3D0E2C8E0E61ADC6555681C4D663396B6D276DA970EC07F3991D91C16F26DFDFD984BC2145526EDADB7F4CE90A9E8B05146C9CDCFB8E583923377ABCF57CEA215BAA19BCF64004EEDBC2FE4E588B4AA7B5CA51CCE30501EBC5976A7135239E8A8D0EC30E0213F070F58DAAE3B39DEB09B4D25CCBB47682CD24B02BD1B5F5D7CBC"
};

//const std::string Galileo_E1_C_PRIMARY_CODE[Galileo_E1_NUMBER_OF_CODES] = {


//const std::string Galileo_E1_C_SECONDARY_CODE = "0011100000001010110110010";
const std::string Compass_NH_SECONDARY_CODE = "00000100110101001110";

#endif /* GNSS_SDR_COMPASS_E1_H_ */
