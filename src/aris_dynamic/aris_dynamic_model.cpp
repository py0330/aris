#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>

#include "aris_core.h"
#include "aris_dynamic_kernel.h"
#include "aris_dynamic_model.h"

namespace Aris
{
	namespace Dynamic
	{
		auto Element::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);
			xml_ele.SetAttribute("type", this->typeName().c_str());
		}
		auto Element::save(const std::string &name)->void
		{
			std::map<std::string, std::shared_ptr<Element> > tem = std::move(save_data_map_);
			auto save_content = std::shared_ptr<Element>(model().typeInfoMap().find(this->typeName())->second.newFromElement(*this));
			tem.insert(std::make_pair(name, save_content));
			save_data_map_ = std::move(tem);
		}
		auto Element::load(const std::string &name)->void
		{
			std::map<std::string, std::shared_ptr<Element> > tem = std::move(save_data_map_);
			model().typeInfoMap().find(this->typeName())->second.assignOperator(*this, *tem.at(name));
			tem.erase(name);
			save_data_map_ = std::move(tem);
		}
		DynEle::DynEle(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id) :Element(father, xml_ele, id)
		{
			if (xml_ele.Attribute("active"))
			{
				if (xml_ele.Attribute("active", "true"))active_ = true;
				else if (xml_ele.Attribute("active", "false"))active_ = false;
				else throw std::runtime_error(std::string("Element \"") + xml_ele.name() + "\" must have valid attibute of Active");
			}
			else
			{
				active_ = true;
			}
		}
		auto DynEle::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);
			xml_ele.SetAttribute("active", active() ? "true" : "false");
		};
		Coordinate::Coordinate(Object &father, const std::string &name, std::size_t id, const double *pm, bool active)
			:DynEle(father, name, id, active)
		{
			static const double default_pm[16]{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,1
			};

			pm = pm ? pm : default_pm;
			setPm(pm);
		}
		Interaction::Interaction(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: DynEle(father, xml_ele, id)
		{
			if (!xml_ele.Attribute("prt_m"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must have Attribute \"prt_m\"");
			if (!model().partPool().find(xml_ele.Attribute("prt_m")))
				throw std::runtime_error(std::string("can't find part m for element \"") + this->name() + "\"");

			if (!xml_ele.Attribute("mak_i"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must have Attribute \"mak_i\"");
			if (!(makI_ = model().partPool().find(xml_ele.Attribute("prt_m"))->markerPool().find(xml_ele.Attribute("mak_i"))))
				throw std::runtime_error(std::string("can't find marker i for element \"") + this->name() + "\"");

			if (!xml_ele.Attribute("prt_n"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must have Attribute \"prt_n\"");
			if (!model().partPool().find(xml_ele.Attribute("prt_n")))
				throw std::runtime_error(std::string("can't find part n for element \"") + this->name() + "\"");

			if (!xml_ele.Attribute("mak_j"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must have Attribute \"mak_j\"");
			if (!(makJ_ = model().partPool().find(xml_ele.Attribute("prt_n"))->markerPool().find(xml_ele.Attribute("mak_j"))))
				throw std::runtime_error(std::string("can't find marker j for element \"") + this->name() + "\"");
		}
		auto Interaction::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);

			xml_ele.SetAttribute("prt_m", this->makI().fatherPart().name().c_str());
			xml_ele.SetAttribute("prt_n", this->makJ().fatherPart().name().c_str());
			xml_ele.SetAttribute("mak_i", this->makI().name().c_str());
			xml_ele.SetAttribute("mak_j", this->makJ().name().c_str());
		}
		auto Constraint::update()->void
		{
			double pm_M2N[4][4];
			double _tem_v1[6]{ 0 }, _tem_v2[6]{ 0 };

			/* Get pm M2N */
			s_pm_dot_pm(*makJ().fatherPart().invPm(), *makI().fatherPart().pm(), *pm_M2N);

			/*update CstMtx*/
			std::fill_n(this->csmJ(), this->dim() * 6, 0);
			s_tf_n(dim(), -1, *pm_M2N, this->csmI(), 0, this->csmJ());

			/*update CstAcc*/
			std::fill_n(this->csa(), this->dim(), 0);
			s_inv_tv(-1, *pm_M2N, makJ().fatherPart().prtVel(), 0, _tem_v1);
			s_cv(makI().fatherPart().prtVel(), _tem_v1, _tem_v2);
			s_dgemmTN(dim(), 1, 6, 1, csmI(), dim(), _tem_v2, 1, 0, csa(), 1);
		}
		auto Constraint::saveAdams(std::ofstream &file) const->void
		{
			file << "constraint create joint " << this->adamsTypeName() << "  &\r\n"
				<< "    joint_name = ." << model().name() << "." << this->name() << "  &\r\n"
				<< "    adams_id = " << adamsID() << "  &\r\n"
				<< "    i_marker_name = ." << model().name() << "." << this->makI().fatherPart().name() << "." << this->makI().name() << "  &\r\n"
				<< "    j_marker_name = ." << model().name() << "." << this->makJ().fatherPart().name() << "." << this->makJ().name() << "  \r\n"
				<< "!\r\n";
		}

		ElementPool<Marker>::ElementPool(Object &father, const Aris::Core::XmlElement &xml_ele) :Object(father, xml_ele)
		{
			for (auto ele = xml_ele.FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				add(*ele);
			}
		};
		auto ElementPool<Marker>::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);
			for (auto &ele : element_vec_)
			{
				auto xml_iter = xml_ele.GetDocument()->NewElement("");
				ele->saveXml(*xml_iter);
				xml_ele.InsertEndChild(xml_iter);
			}
		}
		auto ElementPool<Marker>::saveAdams(std::ofstream &file) const->void
		{
			for (auto &ele : element_vec_)ele->saveAdams(file);
		}
		auto ElementPool<Marker>::save(const std::string &name)->void { for (auto &ele : element_vec_)ele->save(name); };
		auto ElementPool<Marker>::load(const std::string &name)->void { for (auto &ele : element_vec_)ele->load(name); };
		auto ElementPool<Marker>::add(const std::string &name, const double *prt_pm, Marker *relative_mak, bool active)->Marker&
		{
			if (&father() == static_cast<Object *>(&model())) throw std::runtime_error("you can only add marker on part");
			if (find(name))throw std::runtime_error("element \"" + name + "\" already exists, can't add()");
			auto ret = new Marker(this->father(), name, model().markerPool().size(), prt_pm, relative_mak, active);
			element_vec_.push_back(std::shared_ptr<Marker>(ret));
			model().markerPool().element_vec_.push_back(element_vec_.back());
			return std::ref(*ret);
		}
		auto ElementPool<Marker>::add(const Aris::Core::XmlElement &xml_ele)->Marker&
		{
			if (&father() == &model()) throw std::runtime_error("you can only add marker on part");
			if (find(xml_ele.name()))throw std::runtime_error(Marker::TypeName() + " \"" + xml_ele.name() + "\" already exists, can't add element");
			std::string type = xml_ele.Attribute("type") ? xml_ele.Attribute("type") : Marker::TypeName();
			if (model().typeInfoMap().find(type) == model().typeInfoMap().end())
				throw std::runtime_error(std::string("can't find type ") + type);

			auto new_ele = model().typeInfoMap().at(type).newFromXml(this->father(), xml_ele, model().markerPool().size());
			if (!dynamic_cast<Marker*>(new_ele))
			{
				delete new_ele;
				throw std::runtime_error("can't add \"" + type + "\" element to " + Marker::TypeName() + " group");
			}
			element_vec_.push_back(std::shared_ptr<Marker>(dynamic_cast<Marker*>(new_ele)));
			model().markerPool().element_vec_.push_back(element_vec_.back());

			return std::ref(*element_vec_.back().get());
		};
		auto ElementPool<Marker>::at(std::size_t id) ->Marker& { return std::ref(*element_vec_.at(id).get()); };
		auto ElementPool<Marker>::at(std::size_t id) const->const Marker&{ return std::ref(*element_vec_.at(id).get()); };
		auto ElementPool<Marker>::find(const std::string &name)->Marker *
		{
			auto p = std::find_if(element_vec_.begin(), element_vec_.end(), [&name](decltype(element_vec_)::const_reference p)
			{
				return (p->name() == name);
			});

			return p == element_vec_.end() ? nullptr : p->get();
		}
		auto ElementPool<Marker>::find(const std::string &name) const->const Marker *{ return const_cast<ElementPool *>(this)->find(name); }
		auto ElementPool<Marker>::size() const ->std::size_t { return element_vec_.size(); };
		auto ElementPool<Marker>::begin()->std::vector<std::shared_ptr<Marker>>::iterator { return element_vec_.begin(); };
		auto ElementPool<Marker>::begin() const ->std::vector<std::shared_ptr<Marker>>::const_iterator { return element_vec_.begin(); };
		auto ElementPool<Marker>::end()->std::vector<std::shared_ptr<Marker>>::iterator { return element_vec_.end(); };
		auto ElementPool<Marker>::end() const ->std::vector<std::shared_ptr<Marker>>::const_iterator { return element_vec_.end(); };
		auto ElementPool<Marker>::clear() -> void { element_vec_.clear(); };

		Environment::Environment(Object &father, const Aris::Core::XmlElement &xml_ele)
			:Object(father, xml_ele)
		{
			if (!xml_ele.Attribute("gravity"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must has Attribute \"gravity\"");
			try
			{
				Core::Matrix m = this->model().calculator().calculateExpression(xml_ele.Attribute("gravity"));
				if (m.size() != 6)throw std::runtime_error("");
				std::copy_n(m.data(), 6, gravity_);
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute gravity must has valid expression"); }
		}
		auto Environment::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);
			xml_ele.SetAttribute("gravity", Core::Matrix(1, 6, gravity_).toString().c_str());
		}
		auto Environment::saveAdams(std::ofstream &file) const->void
		{
			file << "!-------------------------- Default Units for Model ---------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults units  &\r\n"
				<< "    length = meter  &\r\n"
				<< "    angle = rad  &\r\n"
				<< "    force = newton  &\r\n"
				<< "    mass = kg  &\r\n"
				<< "    time = sec\r\n"
				<< "!\n"
				<< "defaults units  &\r\n"
				<< "    coordinate_system_type = cartesian  &\r\n"
				<< "    orientation_type = body313\r\n"
				<< "!\r\n"
				<< "!------------------------ Default Attributes for Model ------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults attributes  &\r\n"
				<< "    inheritance = bottom_up  &\r\n"
				<< "    icon_visibility = off  &\r\n"
				<< "    grid_visibility = off  &\r\n"
				<< "    size_of_icons = 5.0E-002  &\r\n"
				<< "    spacing_for_grid = 1.0\r\n"
				<< "!\r\n"
				<< "!------------------------------ Adams/View Model ------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "model create  &\r\n"
				<< "   model_name = " << this->model().name() << "\r\n"
				<< "!\r\n"
				<< "view erase\r\n"
				<< "!\r\n"
				<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "force create body gravitational  &\r\n"
				<< "    gravity_field_name = gravity  &\r\n"
				<< "    x_component_gravity = " << this->gravity_[0] << "  &\r\n"
				<< "    y_component_gravity = " << this->gravity_[1] << "  &\r\n"
				<< "    z_component_gravity = " << this->gravity_[2] << "\r\n"
				<< "!\r\n";
		};

		struct Akima::Imp 
		{
			std::vector<double> x_, y_;
			std::vector<double> _p0;
			std::vector<double> _p1;
			std::vector<double> _p2;
			std::vector<double> _p3;
		};
		Akima::~Akima() {};
		Akima::Akima(Object &father, const std::string &name, std::size_t id, int num, const double *x_in, const double *y_in)
			: Element(father, name, id)
		{
			std::list<std::pair<double, double> > data_list;

			for (int i = 0; i < num; ++i)
			{
				data_list.push_back(std::make_pair(x_in[i], y_in[i]));
			}

			this->operator=(Akima(father, name, id, data_list));
		}
		Akima::Akima(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id): Element(father, xml_ele, id) 
		{
			if (!xml_ele.Attribute("x"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must has Attribute \"x\"");
			Core::Matrix mat_x, mat_y;
			try
			{
				mat_x = this->model().calculator().calculateExpression(xml_ele.Attribute("x"));
				mat_y = this->model().calculator().calculateExpression(xml_ele.Attribute("y"));
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute x,y must has valid expression"); }

			*this = Akima(father, xml_ele.name(), id, std::list<double>(mat_x.begin(), mat_x.end()), std::list<double>(mat_y.begin(), mat_y.end()));
		};
		Akima::Akima(Object &father, const std::string &name, std::size_t id, const std::list<double> &x_in, const std::list<double> &y_in)
			: Element(father, name, id)
		{
			if (x_in.size() != y_in.size())throw std::runtime_error("input x and y must have same length");

			std::list<std::pair<double, double> > data_list;

			auto pX = x_in.begin();
			auto pY = y_in.begin();

			for (std::size_t i = 0; i < x_in.size(); ++i)
			{
				data_list.push_back(std::make_pair(*pX, *pY));
				++pX;
				++pY;
			}
			
			this->operator=(Akima(father, name, id, data_list));
		}
		Akima::Akima(Object &father, const std::string &name, std::size_t id, const std::list<std::pair<double, double> > &data_in)
			: Element(father, name, id)
		{
			auto data_list = data_in;
			
			if (data_list.size() < 4)throw std::runtime_error("Akima must be inited with data size more than 4");

			/*对数据进行排序,并保存*/
			data_list.sort([](const std::pair<double, double> &a, const std::pair<double, double> &b)
			{
				return a.first < b.first;
			});

			for (auto &p : data_list)
			{
				imp->x_.push_back(p.first);
				imp->y_.push_back(p.second);
			}

			/*开始计算*/
			std::vector<double> s(data_list.size() + 3), ds(data_list.size() + 2), t(data_list.size());

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				s[i + 2] = (imp->y_[i + 1] - imp->y_[i]) / (imp->x_[i + 1] - imp->x_[i]);
			}

			s[1] = 2 * s[2] - s[3];
			s[0] = 2 * s[1] - s[2];
			s[data_list.size() + 1] = 2 * s[data_list.size()] - s[data_list.size() - 1];
			s[data_list.size() + 2] = 2 * s[data_list.size() + 1] - s[data_list.size()];

			for (std::size_t i = 0; i < data_list.size() + 2; ++i)
			{
				ds[i] = std::abs(s[i + 1] - s[i]);
			}

			for (std::size_t i = 0; i < data_list.size(); ++i)
			{
				if (ds[i] + ds[i + 2]<1e-12)/*前后两段的斜斜率都为0*/
				{
					t[i] = (s[i + 1] + s[i + 2]) / 2;
				}
				else
				{
					t[i] = (ds[i + 2] * s[i + 1] + ds[i] * s[i + 2]) / (ds[i] + ds[i + 2]);
				}
			}

			/*所需储存的变量*/
			imp->_p0.resize(data_list.size() - 1);
			imp->_p1.resize(data_list.size() - 1);
			imp->_p2.resize(data_list.size() - 1);
			imp->_p3.resize(data_list.size() - 1);

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				imp->_p0[i] = imp->y_[i];
				imp->_p1[i] = t[i];
				imp->_p2[i] = (3 * s[i + 2] - 2 * t[i] - t[i + 1]) / (imp->x_[i + 1] - imp->x_[i]);
				imp->_p3[i] = (t[i] + t[i + 1] - 2 * s[i + 2]) / (imp->x_[i + 1] - imp->x_[i]) / (imp->x_[i + 1] - imp->x_[i]);
			}
		}
		Akima::Akima(Object &father, const std::string &name, std::size_t id, const std::list<std::pair<double, double> > &data_in, double begin_slope, double end_slope)
			: Element(father, name, id)
		{
			auto data_list = data_in;

			if (data_list.size() < 4)throw std::runtime_error("Akima must be inited with data size more than 4");

			/*对数据进行排序,并保存*/
			data_list.sort([](const std::pair<double, double> &a, const std::pair<double, double> &b)
			{
				return a.first < b.first;
			});

			for (auto &p : data_list)
			{
				imp->x_.push_back(p.first);
				imp->y_.push_back(p.second);
			}

			/*开始计算*/
			std::vector<double> s(data_list.size() + 3), ds(data_list.size() + 2), t(data_list.size());

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				s[i + 2] = (imp->y_[i + 1] - imp->y_[i]) / (imp->x_[i + 1] - imp->x_[i]);
			}
			///////// this part is different
			s[1] = begin_slope;
			s[0] = begin_slope;
			s[data_list.size() + 1] = end_slope;
			s[data_list.size() + 2] = end_slope;
			///////// this part is different end
			for (std::size_t i = 0; i < data_list.size() + 2; ++i)
			{
				ds[i] = std::abs(s[i + 1] - s[i]);
			}

			for (std::size_t i = 0; i < data_list.size(); ++i)
			{
				if (ds[i] + ds[i + 2]<1e-12)/*前后两段的斜斜率都为0*/
				{
					t[i] = (s[i + 1] + s[i + 2]) / 2;
				}
				else
				{
					t[i] = (ds[i + 2] * s[i + 1] + ds[i] * s[i + 2]) / (ds[i] + ds[i + 2]);
				}
			}

			/*所需储存的变量*/
			imp->_p0.resize(data_list.size() - 1);
			imp->_p1.resize(data_list.size() - 1);
			imp->_p2.resize(data_list.size() - 1);
			imp->_p3.resize(data_list.size() - 1);

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				imp->_p0[i] = imp->y_[i];
				imp->_p1[i] = t[i];
				imp->_p2[i] = (3 * s[i + 2] - 2 * t[i] - t[i + 1]) / (imp->x_[i + 1] - imp->x_[i]);
				imp->_p3[i] = (t[i] + t[i + 1] - 2 * s[i + 2]) / (imp->x_[i + 1] - imp->x_[i]) / (imp->x_[i + 1] - imp->x_[i]);
			}
		}
		auto Akima::saveAdams(std::ofstream &file) const->void
		{
			file << "data_element create spline &\r\n"
				<< "    spline_name = ." << model().name() + "." + name() + " &\r\n"
				<< "    adams_id = " << adamsID() << "  &\r\n"
				<< "    units = m &\r\n"
				<< "    x = " << x().at(0);
			for (auto p = x().begin() + 1; p < x().end(); ++p)
			{
				file << "," << *p;
			}
			file << "    y = " << y().at(0);
			for (auto p = y().begin() + 1; p < y().end(); ++p)
			{
				file << "," << *p;
			}
			file << " \r\n!\r\n";
		}
		auto Akima::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);

			Aris::Core::Matrix mat_x(1, x().size(), imp->x_.data());
			Aris::Core::Matrix mat_y(1, x().size(), imp->y_.data());
			xml_ele.SetAttribute("x", mat_x.toString().c_str());
			xml_ele.SetAttribute("y", mat_y.toString().c_str());
		}
		auto Akima::x() const->const std::vector<double> & { return imp->x_; };
		auto Akima::y() const->const std::vector<double> & { return imp->y_; };
		auto Akima::operator()(double x, char order) const->double
		{
			/*寻找第一个大于x的位置*/
			auto bIn = std::upper_bound(imp->x_.begin(), imp->x_.end() - 1, x);

			int id = std::max<int>(bIn - imp->x_.begin() - 1, 0);

			double w = x - imp->x_[id];

			switch (order)
			{
			case '1':
				return (3 * w*imp->_p3[id] + 2 * imp->_p2[id])*w + imp->_p1[id];
			case '2':
				return (6 * w*imp->_p3[id] + 2 * imp->_p2[id]);
			case '0':
			default:
				return ((w*imp->_p3[id] + imp->_p2[id])*w + imp->_p1[id])*w + imp->_p0[id];
			}
		}
		auto Akima::operator()(int length, const double *x_in, double *y_out, char order)const->void
		{
			for (int i = 0; i < length; ++i)
			{
				y_out[i] = this->operator()(x_in[i], order);
			}
		}

		struct Script::Imp
		{
			struct Node
			{
				virtual ~Node() = default;
				virtual auto doNode()->void {};
				virtual auto adamsScript()const->std::string = 0;
				virtual auto msConsumed()const->std::uint32_t { return 0; };

				static auto create(Model *model, const std::string &expression)->Node*
				{
					std::string split_exp = expression;
					
					std::string delim_str("/,=");
					for (auto delim : delim_str)
					{
						std::regex delim_regex(std::string("") + delim);
						std::string replace = std::string(" ") + delim + std::string(" ");
						split_exp = std::regex_replace(split_exp, delim_regex, replace);
					}

					std::stringstream stream(split_exp);
					std::string word;
					std::size_t id;

					if(!(stream >> word))return nullptr;

					if (word == "activate")
					{
						std::string type;
						stream >> word;
						stream >> type;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> id;
						if (type == "joint")return new ActNode(model->jointPool().at(id - 1), true);
						else if (type == "sforce")return new ActNode(model->forcePool().at(id - 1), true);
						else if (type == "motion")return new ActNode(model->motionPool().at(id - 1), true);
						else throw std::runtime_error("unrecognized deactivate element type");
					}
					else if (word == "deactivate")
					{
						std::string type;
						stream >> word;
						stream >> type;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> id;
						if (type == "joint")return new ActNode(model->jointPool().at(id - 1), false);
						else if (type == "sforce")return new ActNode(model->forcePool().at(id - 1), false);
						else if (type == "motion")return new ActNode(model->motionPool().at(id - 1), false);
						else throw std::runtime_error("unrecognized deactivate element type");
					}
					else if (word == "marker")
					{
						stream >> word;
						stream >> id;
						stream >> word;
						stream >> word;
						stream >> word;
						double prt_pe[6];
						stream >> prt_pe[0];
						stream >> word;
						stream >> prt_pe[1];
						stream >> word;
						stream >> prt_pe[2];
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> prt_pe[3];
						stream >> word;
						stream >> prt_pe[4];
						stream >> word;
						stream >> prt_pe[5];
						stream >> word;
						return new AlnNode(model->markerPool().at(id - 1), prt_pe);
					}
					else if (word == "simulate")
					{
						double dur, dt;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> dur;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> dt;

						return new SimNode(static_cast<std::uint32_t>(dur * 1000), static_cast<std::uint32_t>(dt * 1000));
					}
					else
					{
						throw std::runtime_error("failed parse script");
					}
				}
			};
			struct ActNode final :public Node
			{
				virtual auto doNode()->void override final { dyn_ele_->activate(isActive); };
				virtual auto adamsScript()const->std::string override final
				{
					std::stringstream ss;
					std::string cmd = isActive ? "activate/" : "deactivate/";
					ss << cmd << dyn_ele_->adamsGroupName() << ", id=" << dyn_ele_->adamsID();
					return std::move(ss.str());
				};
				explicit ActNode(DynEle &ele, bool isActive) :dyn_ele_(&ele), isActive(isActive) {};

				bool isActive;
				DynEle *dyn_ele_;
			};
			struct AlnNode final :public Node
			{
				virtual auto doNode()->void override final
				{
					if (mak_target_)
					{
						double pm_target_g[16];

						s_pm_dot_pm(*mak_target_->fatherPart().pm(), *mak_target_->prtPm(), pm_target_g);
						s_inv_pm_dot_pm(*mak_move_->fatherPart().pm(), pm_target_g, const_cast<double *>(&mak_move_->prtPm()[0][0]));
						s_pm2pe(*mak_move_->prtPm(), prt_pe_);
					}
				};
				virtual auto adamsScript()const->std::string override final
				{
					std::stringstream ss;
					ss << std::setprecision(15) << "marker/" << mak_move_->adamsID()
						<< " , QP = " << prt_pe_[0] << "," << prt_pe_[1] << "," << prt_pe_[2]
						<< " , REULER =" << prt_pe_[3] << "," << prt_pe_[4] << "," << prt_pe_[5];
					return std::move(ss.str());
				};
				explicit AlnNode(Marker &mak_move, const Marker &mak_target) :mak_move_(&mak_move), mak_target_(&mak_target) {};
				explicit AlnNode(Marker &mak_move, const double *prt_pe) :mak_move_(&mak_move), mak_target_(nullptr) 
				{
					std::copy_n(prt_pe, 6, prt_pe_);
				};
				
				Marker *mak_move_;
				const Marker *mak_target_;
				double prt_pe_[6];
			};
			struct SimNode final :public Node
			{
				virtual auto msConsumed()const->std::uint32_t override final { return ms_dur_; };
				virtual auto adamsScript()const->std::string override final
				{
					std::stringstream ss;
					ss << "simulate/transient, dur=" << double(ms_dur_) / 1000.0 << ", dtout=" << double(ms_dt_) / 1000.0;
					return std::move(ss.str());
				};

				explicit SimNode(std::uint32_t ms_dur, std::uint32_t ms_dt) :ms_dur_(ms_dur), ms_dt_(ms_dt) { };
				std::uint32_t ms_dur_;
				std::uint32_t ms_dt_;
			};

			Imp(Model *model) :model_(model) {};
			Imp(const Imp &other) :model_(other.model_) 
			{
				for (auto &node : other.node_list_)
				{
					this->node_list_.push_back(std::unique_ptr<Node>(Node::create(model_, node->adamsScript())));
				}
			};
			auto operator=(const Imp& other)->Imp&
			{
				this->node_list_.clear();
				for (auto &node : other.node_list_)
				{
					this->node_list_.push_back(std::unique_ptr<Node>(Node::create(model_, node->adamsScript())));
				}

				return *this;
			}

			std::list<std::unique_ptr<Node> > node_list_;
			Model *model_;
		};
		Script::~Script() {};
		Script::Script(Object &father, const std::string &name, std::size_t id) :Element(father, name, id), imp(&model()) {};
		Script::Script(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id) :Element(father, xml_ele, id), imp(&model())
		{
			std::stringstream stream(xml_ele.GetText());
			std::string line;
			while (std::getline(stream, line))
			{
				line.erase(0, line.find_first_not_of(" "));
				line.erase(line.find_last_not_of(" ") + 1);
				if (line != "")imp->node_list_.push_back(std::unique_ptr<Imp::Node>(Imp::Node::create(&model(), line)));
			}
		};
		auto Script::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);
			
			std::stringstream stream;
			stream << "\n";
			for (auto &node : imp->node_list_)
			{
				stream << node->adamsScript() << "\n";
			}

			xml_ele.SetText(stream.str().c_str());
		};
		auto Script::saveAdams(std::ofstream &file) const->void
		{
			file << "simulation script create &\r\n"
				<< "sim_script_name = default_script &\r\n"
				<< "solver_commands = ";

			for (auto &node : imp->node_list_)
			{
				file << "&\r\n\"" << node->adamsScript() << "\",";
			}

			file << "\"\"\r\n\r\n";
		}
		auto Script::act(DynEle &ele, bool active)->void 
		{
			imp->node_list_.push_back(std::unique_ptr<Imp::Node>(new Imp::ActNode(ele, active)));
		};
		auto Script::aln(Marker &mak_move, const Marker& mak_target)->void 
		{
			imp->node_list_.push_back(std::unique_ptr<Imp::Node>(new Imp::AlnNode(mak_move, mak_target)));
		};
		auto Script::sim(std::uint32_t ms_dur, std::uint32_t ms_dt)->void 
		{
			imp->node_list_.push_back(std::unique_ptr<Imp::Node>(new Imp::SimNode(ms_dur, ms_dt)));
		};
		auto Script::empty() const->bool { return imp->node_list_.empty(); };
		auto Script::endTime()const->std::uint32_t 
		{
			std::uint32_t end_time{ 0 };
			for (auto& node : imp->node_list_)end_time += node->msConsumed();
			return end_time;
		};
		auto Script::doScript(std::uint32_t ms_begin, std::uint32_t ms_end)->void
		{
			std::uint32_t now{ 0 };
			for (auto& node : imp->node_list_)
			{
				if ((now >= ms_begin) && (now < ms_end)) node->doNode();
				if ((now += node->msConsumed()) >= ms_end)break;
			}
		}
		auto Script::clear()->void { return imp->node_list_.clear(); };

		struct Marker::Imp
		{
			double prt_pm_[4][4];
		};
		Marker::~Marker() {};
		Marker::Marker(Object &father, const std::string &name, std::size_t id, const double *prt_pm, Marker *relative_mak, bool active)
			: Coordinate(father, name, id, nullptr, active)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			prt_pm = prt_pm ? prt_pm : default_pm_in;

			if (relative_mak)
			{
				if (&relative_mak->fatherPart() != &fatherPart())
					throw std::logic_error("relative marker must has same father part with this marker");

				s_pm_dot_pm(*relative_mak->prtPm(), prt_pm, *imp->prt_pm_);
			}
			else
			{
				std::copy_n(prt_pm, 16, static_cast<double *>(*imp->prt_pm_));
			}
		}
		Marker::Marker(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: Coordinate(father, xml_ele, id)
		{
			double pm[16];

			if (!xml_ele.Attribute("pe"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must have Attribute \"Pos\"");
			try
			{
				Core::Matrix m = this->model().calculator().calculateExpression(xml_ele.Attribute("pe"));
				if (m.size() != 6)throw std::runtime_error("");
				s_pe2pm(m.data(), pm);
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute \"pe\" must be a matrix expression"); }

			if (xml_ele.Attribute("relative_to"))
			{
				try { s_pm_dot_pm(*fatherPart().markerPool().find(xml_ele.Attribute("relative_to"))->prtPm(), pm, *imp->prt_pm_); }
				catch (std::exception &) { throw std::runtime_error(std::string("can't find relative marker for element \"") + this->name() + "\""); }
			}
			else
			{
				std::copy_n(pm, 16, static_cast<double*>(*imp->prt_pm_));
			}
		}
		auto Marker::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);
			double pe[6];
			s_pm2pe(*prtPm(), pe);
			xml_ele.SetAttribute("pe", Core::Matrix(1, 6, pe).toString().c_str());
		}
		auto Marker::saveAdams(std::ofstream &file) const->void
		{
			double pe[6];

			s_pm2pe(*prtPm(), pe, "313");
			Core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

			file << "marker create  &\r\n"
				<< "marker_name = ." << model().name() << "." << father().name() << "." << this->name() << "  &\r\n"
				<< "adams_id = " << adamsID()<< "  &\r\n"
				<< "location = (" << loc.toString() << ")  &\r\n"
				<< "orientation = (" << ori.toString() << ")\r\n"
				<< "!\r\n";
		}
		auto Marker::fatherPart() const->const Part&{ return static_cast<const Part &>(this->father()); };
		auto Marker::fatherPart()->Part&{ return static_cast<Part &>(this->father()); };
		auto Marker::vel() const->const double6&{ return fatherPart().vel(); };
		auto Marker::acc() const->const double6&{ return fatherPart().acc(); };
		auto Marker::prtPm() const->const double4x4&{ return imp->prt_pm_; };
		auto Marker::update()->void
		{
			s_pm_dot_pm(*fatherPart().pm(), *prtPm(), *pm());
		}

		struct Part::Imp
		{
			Imp(Object &father) :marker_pool_(father, "ChildMarker") {};
			
			ElementPool<Marker> marker_pool_;
			
			double inv_pm_[4][4]{ { 0 } };
			double vel_[6]{ 0 };
			double acc_[6]{ 0 };

			double prt_im_[6][6]{ { 0 } };
			double prt_gravity_[6]{ 0 };
			double prt_acc_[6]{ 0 };
			double prt_vel_[6]{ 0 };
			double prt_fg_[6]{ 0 };
			double prt_fv_[6]{ 0 };

			int row_id_;
			std::string graphic_file_path_;
		};
		Part::~Part() {};
		Part::Part(Object &father, const std::string &name, std::size_t id, const double *im, const double *pm, const double *vel, const double *acc, bool active)
			: Coordinate(father, name, id, nullptr, active), imp(std::ref(*this))
		{
			static const double default_im[36]{
				1,0,0,0,0,0,
				0,1,0,0,0,0,
				0,0,1,0,0,0,
				0,0,0,1,0,0,
				0,0,0,0,1,0,
				0,0,0,0,0,1,
			};

			static const double default_vel[6]{ 0,0,0,0,0,0 };
			static const double default_acc[6]{ 0,0,0,0,0,0 };
			
			im = im ? im : default_im;
			vel = vel ? vel : default_vel;
			acc = acc ? acc : default_acc;
			
			std::copy_n(im, 36, static_cast<double *>(*imp->prt_im_));
			setVel(vel);
			setAcc(acc);
		}
		Part::Part(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: Coordinate(father, xml_ele, id), imp(std::ref(*this))
		{
			try
			{
				auto m = this->model().calculator().calculateExpression(xml_ele.Attribute("pe"));
				if (m.size() != 6)throw std::runtime_error("");
				s_pe2pm(m.data(), *pm());
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute \"inertia\" must be a matrix expression"); }

			try
			{
				auto m = this->model().calculator().calculateExpression(xml_ele.Attribute("vel"));
				if (m.size() != 6)throw std::runtime_error("");
				std::copy_n(m.data(), 6, vel());
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute \"vel\" must be a matrix expression"); }

			try
			{
				auto m = this->model().calculator().calculateExpression(xml_ele.Attribute("acc"));
				if (m.size() != 6)throw std::runtime_error("");
				std::copy_n(m.data(), 6, acc());
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute \"acc\" must be a matrix expression"); }
		
			try
			{
				auto m = this->model().calculator().calculateExpression(xml_ele.Attribute("inertia"));
				if (m.size() != 10)throw std::runtime_error("");
				s_gamma2im(m.data(), *imp->prt_im_);
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute \"inertia\" must be a matrix expression"); }


			if (xml_ele.Attribute("graphic_file_path"))
				imp->graphic_file_path_ = model().calculator().evaluateExpression(xml_ele.Attribute("graphic_file_path"));

			auto mak_group_xml = xml_ele.FirstChildElement("ChildMarker");
			for (auto ele = mak_group_xml->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				markerPool().add(*ele);
			}
		}
		auto Part::rowID()const->std::size_t { return imp->row_id_; };
		auto Part::vel()const->const double6&{ return imp->vel_; };
		auto Part::vel()->double6& { return imp->vel_; };
		auto Part::acc()const->const double6&{ return imp->acc_; };
		auto Part::acc()->double6& { return imp->acc_; };
		auto Part::invPm() const->const double4x4&{ return imp->inv_pm_; };
		auto Part::prtIm() const->const double6x6&{ return imp->prt_im_; };
		auto Part::prtVel() const->const double6&{ return imp->prt_vel_; };
		auto Part::prtAcc() const->const double6&{ return imp->prt_acc_; };
		auto Part::prtFg() const->const double6&{ return imp->prt_fg_; };
		auto Part::prtFv() const->const double6&{ return imp->prt_fv_; };
		auto Part::prtGravity() const->const double6&{ return imp->prt_gravity_; };
		auto Part::markerPool()->ElementPool<Marker>& { return std::ref(imp->marker_pool_); };
		auto Part::markerPool()const->const ElementPool<Marker>& { return std::ref(imp->marker_pool_); };
		auto Part::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);
			
			double pe[6];
			s_pm2pe(*pm(), pe);
			xml_ele.SetAttribute("pe", Core::Matrix(1, 6, pe).toString().c_str());
			xml_ele.SetAttribute("vel", Core::Matrix(1, 6, vel()).toString().c_str());
			xml_ele.SetAttribute("acc", Core::Matrix(1, 6, acc()).toString().c_str());
			
			double gamma[10];
			s_im2gamma(*this->prtIm(), gamma);
			xml_ele.SetAttribute("inertia", Core::Matrix(1, 10, gamma).toString().c_str());
			xml_ele.SetAttribute("graphic_file_path", imp->graphic_file_path_.c_str());

			auto child_mak_group = xml_ele.GetDocument()->NewElement("ChildMarker");
			xml_ele.InsertEndChild(child_mak_group);
			markerPool().saveXml(*child_mak_group);
		}
		auto Part::saveAdams(std::ofstream &file) const->void
		{
			if (this == &model().ground())
			{
				file << "!----------------------------------- ground -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "! ****** Ground Part ******\r\n"
					<< "!\r\n"
					<< "defaults model  &\r\n"
					<< "    part_name = ground\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
					<< "!\r\n"
					<< "! ****** Markers for current part ******\r\n"
					<< "!\r\n";
			}
			else
			{
				double pe[6];
				s_pm2pe(*this->pm(), pe, "313");
				Core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "!----------------------------------- " << this->name() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    adams_id = " << this->adamsID() << "  &\r\n"
					<< "    location = (" << loc.toString() << ")  &\r\n"
					<< "    orientation = (" << ori.toString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << "." << this->name() << " \r\n"
					<< "!\r\n";

				
				double mass = this->prtIm()[0][0] == 0 ? 1 : prtIm()[0][0];
				std::fill_n(pe, 6, 0);
				pe[0] = this->prtIm()[1][5] / mass;
				pe[1] = -this->prtIm()[0][5] / mass;
				pe[2] = this->prtIm()[0][4] / mass;				

				file << "! ****** cm and mass for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    adams_id = " << id() + model().markerPool().size() << "  &\r\n"
					<< "    location = ({" << pe[0] << "," << pe[1] << "," << pe[2] << "})  &\r\n"
					<< "    orientation = (" << "{0,0,0}" << ")\r\n"
					<< "!\r\n";

				double pm[16];
				double im[6][6];

				pe[0] = -pe[0];
				pe[1] = -pe[1];
				pe[2] = -pe[2];

				s_pe2pm(pe, pm);
				s_i2i(pm, *this->prtIm(), *im);

				///！注意！///
				//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy，Ixz，Iyz上互为相反数。别问我为什么，我也不知道。
				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    mass = " << this->prtIm()[0][0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    ixx = " << im[3][3] << "  &\r\n"
					<< "    iyy = " << im[4][4] << "  &\r\n"
					<< "    izz = " << im[5][5] << "  &\r\n"
					<< "    ixy = " << -im[4][3] << "  &\r\n"
					<< "    izx = " << -im[5][3] << "  &\r\n"
					<< "    iyz = " << -im[5][4] << "\r\n"
					<< "!\r\n";

				
			}

			//导入marker
			this->markerPool().saveAdams(file);

			//导入parasolid
			std::stringstream stream(imp->graphic_file_path_);
			std::string path;
			while (stream >> path)
			{
				file << "file parasolid read &\r\n"
					<< "file_name = \"" << path << "\" &\r\n"
					<< "type = ASCII" << " &\r\n"
					<< "part_name = " << this->name() << " \r\n"
					<< "\r\n";
			}
		}
		auto Part::update()->void
		{
			double tem[6];

			s_inv_pm(*pm(), *imp->inv_pm_);
			s_tv(*invPm(), vel(), imp->prt_vel_);
			s_tv(*invPm(), acc(), imp->prt_acc_);
			s_tv(*invPm(), model().environment().gravity_, imp->prt_gravity_);
			s_m6_dot_v6(*prtIm(), prtGravity(), imp->prt_fg_);
			s_m6_dot_v6(*prtIm(), imp->prt_vel_, tem);
			s_cf(prtVel(), tem, imp->prt_fv_);
		}
		
		Motion::Motion(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, const double *frc_coe, bool active)
			: Constraint(father, name, id, makI, makJ, active)
		{
			static const double default_frc_coe[3]{ 0,0,0 };

			frc_coe = frc_coe ? frc_coe : default_frc_coe;

			std::copy_n(frc_coe, 3, frc_coe_);
		}
		Motion::Motion(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: Constraint(father, xml_ele, id)
		{
			if(!xml_ele.Attribute("frc_coe"))throw std::runtime_error(std::string("xml element \"") + xml_ele.name() + "\" must have Attribute \"frc_coe\"");
			try 
			{ 
				Core::Matrix m = model().calculator().calculateExpression(xml_ele.Attribute("frc_coe"));
				if (m.size() != 3)throw std::runtime_error("");
				std::copy_n(m.data(), 3, static_cast<double*>(frc_coe_));
			}
			catch (std::exception &) { throw std::runtime_error(std::string("xml element \"") + this->name() + "\" attribute frc_coe must be a matrix expression"); }
		}
		auto Motion::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Constraint::saveXml(xml_ele);
			xml_ele.SetAttribute("frc_coe", Core::Matrix(1, 3, this->frcCoe()).toString().c_str());
		}

		struct Model::Imp
		{
			Imp(Object &father)
				: environment_{ father,"Environment" }
				, script_pool_{ father, "Script" }
				, variable_pool_{ father,"Variable" }
				, akima_pool_{ father,"Akima" }
				, marker_pool_{ father,"Marker" }
				, part_pool_{ father,"Part" }
				, joint_pool_{ father,"Joint" }
				, motion_pool_{ father,"Motion" }
				, force_pool_{ father,"Force" }
				, type_info_map_{
					std::make_pair(MatrixVariable::TypeName(), TypeInfo::create<MatrixVariable>()),
					std::make_pair(StringVariable::TypeName(), TypeInfo::create<StringVariable>()),
					std::make_pair(Akima::TypeName(), TypeInfo::create<Akima>()),
					std::make_pair(Marker::TypeName(), TypeInfo::create<Marker>()),
					std::make_pair(Part::TypeName(), TypeInfo::create<Part>()),
					std::make_pair(RevoluteJoint::TypeName(), TypeInfo::create<RevoluteJoint>()),
					std::make_pair(TranslationalJoint::TypeName(), TypeInfo::create<TranslationalJoint>()),
					std::make_pair(UniversalJoint::TypeName(), TypeInfo::create<UniversalJoint>()),
					std::make_pair(SphericalJoint::TypeName(), TypeInfo::create<SphericalJoint>()),
					std::make_pair(SingleComponentMotion::TypeName(), TypeInfo::create<SingleComponentMotion>()),
					std::make_pair(SingleComponentForce::TypeName(), TypeInfo::create<SingleComponentForce>()),	}
			{
				


			};
			
			std::map<std::string, TypeInfo> type_info_map_;
			Aris::Core::Calculator calculator_;
			Environment environment_;
			ElementPool<Script> script_pool_;
			ElementPool<Variable> variable_pool_;
			ElementPool<Akima> akima_pool_;
			ElementPool<Marker> marker_pool_;
			ElementPool<Part> part_pool_;
			ElementPool<Joint> joint_pool_;
			ElementPool<Motion> motion_pool_;
			ElementPool<Force> force_pool_;

			Part* ground_;

			std::size_t dyn_cst_dim_, dyn_prt_dim_;
			std::size_t clb_dim_m_, clb_dim_n_, clb_dim_gam_, clb_dim_frc_;

			std::function<void(int dim, const double *D, const double *b, double *x)> dyn_solve_method_{ nullptr };
			std::function<void(int n, double *A)> clb_inverse_method_{ nullptr };
		};
		Model::Model(const std::string & name): Object(std::ref(*this), name), imp(std::ref(*this))
		{
			registerElementType<Script>();
			
			partPool().add<Part>("Ground");
		}
		Model::~Model()
		{
		}
		auto Model::load(const std::string &name)->void
		{
			variablePool().load(name);
			akimaPool().load(name);
			scriptPool().load(name);
			markerPool().load(name);
			partPool().load(name);
			jointPool().load(name);
			motionPool().load(name);
			forcePool().load(name);
		}
		auto Model::save(const std::string &name)->void
		{
			variablePool().save(name);
			akimaPool().save(name);
			scriptPool().save(name);
			markerPool().save(name);
			partPool().save(name);
			jointPool().save(name);
			motionPool().save(name);
			forcePool().save(name);
		}
		auto Model::loadDynEle(const std::string &name)->void
		{
			markerPool().load(name);
			partPool().load(name);
			jointPool().load(name);
			motionPool().load(name);
			forcePool().load(name); 
		}
		auto Model::saveDynEle(const std::string &name)->void
		{
			markerPool().save(name);
			partPool().save(name);
			jointPool().save(name);
			motionPool().save(name);
			forcePool().save(name);
		}
		auto Model::loadXml(const std::string &filename)->void
		{
			Aris::Core::XmlDocument xmlDoc;

			if (xmlDoc.LoadFile(filename.c_str()) != 0)
			{
				throw std::runtime_error((std::string("could not open file:") + std::string(filename)));
			}

			loadXml(xmlDoc);
		}
		auto Model::loadXml(const Aris::Core::XmlDocument &xml_doc)->void
		{
			auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");

			if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");

			loadXml(*model_xml_ele);
		}
		auto Model::loadXml(const Aris::Core::XmlElement &xml_ele)->void
		{
			clear();

			auto env_xml_ele = xml_ele.FirstChildElement("Environment");
			if (!env_xml_ele)throw(std::runtime_error("Model must have environment element"));
			imp->environment_ = std::move(Environment(model(), *env_xml_ele));

			auto var_xml_ele = xml_ele.FirstChildElement("Variable");
			if (!var_xml_ele)throw(std::runtime_error("Model must have variable element"));
			imp->variable_pool_ = std::move(ElementPool<Variable>(*this, *var_xml_ele));

			auto aki_xml_ele = xml_ele.FirstChildElement("Akima");
			if (!aki_xml_ele)throw(std::runtime_error("Model must have akima element"));
			imp->akima_pool_ = std::move(ElementPool<Akima>(*this, *aki_xml_ele));

			auto prt_xml_ele = xml_ele.FirstChildElement("Part");
			if (!prt_xml_ele)throw(std::runtime_error("Model must have part element"));
			imp->part_pool_ = std::move(ElementPool<Part>(*this, *prt_xml_ele));

			auto jnt_xml_ele = xml_ele.FirstChildElement("Joint");
			if (!jnt_xml_ele)throw(std::runtime_error("Model must have joint element"));
			imp->joint_pool_ = std::move(ElementPool<Joint>(*this, *jnt_xml_ele));

			auto mot_xml_ele = xml_ele.FirstChildElement("Motion");
			if (!mot_xml_ele)throw(std::runtime_error("Model must have motion element"));
			imp->motion_pool_ = std::move(ElementPool<Motion>(*this, *mot_xml_ele));

			auto fce_xml_ele = xml_ele.FirstChildElement("Force");
			if (!fce_xml_ele)throw(std::runtime_error("Model must have force element"));
			imp->force_pool_ = std::move(ElementPool<Force>(*this, *fce_xml_ele));

			auto sci_xml_ele = xml_ele.FirstChildElement("Script");
			if (!sci_xml_ele)throw(std::runtime_error("Model must have Script element"));
			imp->script_pool_ = std::move(ElementPool<Script>(*this, *sci_xml_ele));

			if (!(imp->ground_ = partPool().find("Ground")))throw std::runtime_error("model must has a part named \"Ground\"");
		}
		auto Model::saveXml(const std::string &filename) const->void
		{
			Aris::Core::XmlDocument doc;

			saveXml(doc);

			doc.SaveFile(filename.c_str());
		}
		auto Model::saveXml(Aris::Core::XmlDocument &xml_doc)const->void
		{
			xml_doc.DeleteChildren();

			auto header_xml_ele = xml_doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			xml_doc.InsertEndChild(header_xml_ele);

			auto root_xml_ele = xml_doc.NewElement("Root");
			xml_doc.InsertEndChild(root_xml_ele);

			auto model_xml_ele = xml_doc.NewElement("Model");
			root_xml_ele->InsertEndChild(model_xml_ele);
			saveXml(*model_xml_ele);
		}
		auto Model::saveXml(Aris::Core::XmlElement &xml_ele)const->void
		{
			xml_ele.SetName(this->name().c_str());
			xml_ele.DeleteChildren();

			auto env_xml_ele = xml_ele.GetDocument()->NewElement("");
			xml_ele.InsertEndChild(env_xml_ele);
			environment().saveXml(*env_xml_ele);

			auto var_xml_ele = xml_ele.GetDocument()->NewElement("Variable");
			xml_ele.InsertEndChild(var_xml_ele);
			variablePool().saveXml(*var_xml_ele);

			auto aki_xml_ele = xml_ele.GetDocument()->NewElement("Akima");
			xml_ele.InsertEndChild(aki_xml_ele);
			akimaPool().saveXml(*aki_xml_ele);

			auto sci_xml_ele = xml_ele.GetDocument()->NewElement("Script");
			xml_ele.InsertEndChild(sci_xml_ele);
			scriptPool().saveXml(*sci_xml_ele);

			auto prt_xml_ele = xml_ele.GetDocument()->NewElement("Part");
			xml_ele.InsertEndChild(prt_xml_ele);
			partPool().saveXml(*prt_xml_ele);

			auto jnt_xml_ele = xml_ele.GetDocument()->NewElement("Joint");
			xml_ele.InsertEndChild(jnt_xml_ele);
			jointPool().saveXml(*jnt_xml_ele);

			auto mot_xml_ele = xml_ele.GetDocument()->NewElement("Motion");
			xml_ele.InsertEndChild(mot_xml_ele);
			motionPool().saveXml(*mot_xml_ele);

			auto fce_xml_ele = xml_ele.GetDocument()->NewElement("Force");
			xml_ele.InsertEndChild(fce_xml_ele);
			forcePool().saveXml(*fce_xml_ele);
		}
		auto Model::saveAdams(const std::string &filename, bool using_script) const->void
		{
			std::string filename_ = filename;
			if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd")
			{
				filename_ += ".cmd";
			}

			std::ofstream file;
			file.open(filename_, std::ios::out | std::ios::trunc);

			saveAdams(file, using_script);

			file.close();
		}
		auto Model::saveAdams(std::ofstream &file, bool using_script) const->void
		{
			file << std::setprecision(15);

			file << "!----------------------------------- Environment -------------------------------!\r\n!\r\n!\r\n";
			environment().saveAdams(file);

			file << "!----------------------------------- Akimas -------------------------------------!\r\n!\r\n!\r\n";
			for (auto &aki : akimaPool())aki->saveAdams(file);

			file << "!----------------------------------- Parts -------------------------------------!\r\n!\r\n!\r\n";
			for (auto &prt : partPool())prt->saveAdams(file);

			file << "!----------------------------------- Joints ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &jnt : jointPool())jnt->saveAdams(file);

			file << "!----------------------------------- Motions -----------------------------------!\r\n!\r\n!\r\n";
			for (auto &mot : motionPool())mot->saveAdams(file);

			file << "!----------------------------------- Forces ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &fce : forcePool())fce->saveAdams(file);

			file << "!----------------------------------- Script ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &sci : scriptPool())sci->saveAdams(file);

			if (!using_script)
			{
				file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
				for (auto &prt : partPool())
				{
					if ((prt.get() != &ground()) && (!prt->active()))
					{
						file << "part attributes  &\r\n"
							<< "    part_name = ." << name() << "." << prt->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
				for (auto &jnt : jointPool())
				{
					if (!jnt->active())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << name() << "." << jnt->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}

				}
				for (auto &mot : motionPool())
				{
					if (!mot->active())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << name() << "." << mot->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}

				}
				for (auto &fce : forcePool())
				{
					if (!fce->active())
					{
						file << "force attributes  &\r\n"
							<< "    force_name = ." << name() << "." << fce->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
			}
		}
		auto Model::clear()->void
		{
			calculator().clearVariables();
			variablePool().clear();
			akimaPool().clear();
			markerPool().clear();
			partPool().clear();
			jointPool().clear();
			motionPool().clear();
			forcePool().clear();
		}
		auto Model::calculator()->Aris::Core::Calculator& { return std::ref(imp->calculator_); }
		auto Model::calculator()const ->const Aris::Core::Calculator&{ return std::ref(imp->calculator_); }
		auto Model::environment()->Aris::Dynamic::Environment& { return std::ref(imp->environment_); };
		auto Model::environment()const ->const Aris::Dynamic::Environment&{ return std::ref(imp->environment_); };
		auto Model::scriptPool()->ElementPool<Script>& { return std::ref(imp->script_pool_); };
		auto Model::scriptPool()const->const ElementPool<Script>&{ return std::ref(imp->script_pool_); };
		auto Model::variablePool()->ElementPool<Variable>& { return std::ref(imp->variable_pool_); };
		auto Model::variablePool()const->const ElementPool<Variable>& { return std::ref(imp->variable_pool_); };
		auto Model::akimaPool()->ElementPool<Akima>& { return std::ref(imp->akima_pool_); };
		auto Model::akimaPool()const->const ElementPool<Akima>& { return std::ref(imp->akima_pool_); };
		auto Model::markerPool()->ElementPool<Marker>& { return std::ref(imp->marker_pool_); };
		auto Model::markerPool()const->const ElementPool<Marker>& { return std::ref(imp->marker_pool_); };
		auto Model::partPool()->ElementPool<Part>& { return std::ref(imp->part_pool_); };
		auto Model::partPool()const->const ElementPool<Part>& { return std::ref(imp->part_pool_); };
		auto Model::jointPool()->ElementPool<Joint>& { return std::ref(imp->joint_pool_); };
		auto Model::jointPool()const->const ElementPool<Joint>& { return std::ref(imp->joint_pool_); };
		auto Model::motionPool()->ElementPool<Motion>& { return std::ref(imp->motion_pool_); };
		auto Model::motionPool()const->const ElementPool<Motion>& { return std::ref(imp->motion_pool_); };
		auto Model::forcePool()->ElementPool<Force>& { return std::ref(imp->force_pool_); };
		auto Model::forcePool()const->const ElementPool<Force>& { return std::ref(imp->force_pool_); };
		auto Model::ground()->Part& { return std::ref(*imp->ground_); };
		auto Model::ground()const->const Part&{ return std::ref(*imp->ground_); };
		auto Model::typeInfoMap()->std::map<std::string, TypeInfo>& { return std::ref(imp->type_info_map_); };
		auto Model::dynDimM()const->std::size_t { return imp->dyn_prt_dim_; };
		auto Model::dynDimN()const->std::size_t { return imp->dyn_cst_dim_; };
		auto Model::dynSetSolveMethod(std::function<void(int dim, const double *D, const double *b, double *x)> solve_method)->void
		{
			imp->dyn_solve_method_ = solve_method;
		}
		auto Model::dynCstMtx(double *cst_mtx) const->void
		{
			std::fill_n(cst_mtx, dynDimN()*dynDimM(), 0);

			for (int i = 0; i < 6; ++i)
			{
				cst_mtx[dynDimN()*(ground().rowID() + i) + i] = 1;
			}

			for (auto &jnt : jointPool())
			{
				if (jnt->active())
				{
					s_block_cpy(6, jnt->dim(), jnt->csmI(), 0, 0, jnt->dim(), cst_mtx, jnt->makI().fatherPart().rowID(), jnt->col_id_, dynDimN());
					s_block_cpy(6, jnt->dim(), jnt->csmJ(), 0, 0, jnt->dim(), cst_mtx, jnt->makJ().fatherPart().rowID(), jnt->col_id_, dynDimN());
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot->active())
				{
					s_block_cpy(6, 1, mot->csmI(), 0, 0, 1, cst_mtx, mot->makI().fatherPart().rowID(), mot->col_id_, dynDimN());
					s_block_cpy(6, 1, mot->csmJ(), 0, 0, 1, cst_mtx, mot->makJ().fatherPart().rowID(), mot->col_id_, dynDimN());
				}
			}
		}
		auto Model::dynIneMtx(double *ine_mtx) const->void
		{
			std::fill_n(ine_mtx, dynDimM()*dynDimM(), 0);

			for (int i = 0; i < 6; ++i)
			{
				ine_mtx[dynDimM()*(ground().rowID() + i) + ground().rowID() + i] = 1;
			}

			for (auto &prt : partPool())
			{
				if (prt->active())
				{
					s_block_cpy(6, 6, *(prt->prtIm()), 0, 0, 6, ine_mtx, prt->rowID(), prt->rowID(), dynDimM());
				}
			}
		}
		auto Model::dynCstAcc(double *cst_acc) const->void
		{
			std::fill_n(cst_acc, dynDimN(), 0);

			for (auto &jnt : jointPool())
			{
				if (jnt->active())
				{
					std::copy_n(jnt->csa(), jnt->dim(), &cst_acc[jnt->col_id_]);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot->active())
				{
					cst_acc[mot->col_id_] = *mot->csa();
				}
			}
		}
		auto Model::dynPrtFce(double *prt_fce) const->void
		{
			std::fill_n(prt_fce, dynDimM(), 0);

			for (auto &prt : partPool())
			{
				if (prt->active())
				{
					s_daxpy(6, -1, prt->prtFg(), 1, &prt_fce[prt->rowID()], 1);
					s_daxpy(6, 1, prt->prtFv(), 1, &prt_fce[prt->rowID()], 1);
				}
			}

			for (auto &fce : forcePool())
			{
				if (fce->active())
				{
					s_daxpy(6, -1, fce->fceI(), 1, &prt_fce[fce->makI().fatherPart().rowID()], 1);
					s_daxpy(6, -1, fce->fceJ(), 1, &prt_fce[fce->makJ().fatherPart().rowID()], 1);
				}
			}
		}
		auto Model::dynCstFce(double *cst_fce) const->void
		{
			for (auto &jnt : jointPool())
			{
				if (jnt->active())
				{
					std::copy_n(jnt->csf(), jnt->dim(), &cst_fce[jnt->col_id_]);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot->active())
				{
					cst_fce[mot->col_id_] = mot->motFceDyn();
				}
			}

		}
		auto Model::dynPrtAcc(double *cst_acc) const->void
		{
			for (auto &prt : partPool())
			{
				if (prt->active())
				{
					std::copy_n(prt->prtAcc(), 6, &cst_acc[prt->rowID()]);
				}
			}
		}
		auto Model::dynPre()->void
		{
			std::size_t pid = 0;//part id
			std::size_t cid = 6;//Joint id

			for (auto &prt:partPool())
			{
				prt->imp->row_id_ = prt->active() ? (pid += 6) - 6 : 0;
			}
			for (auto &jnt:jointPool())
			{
				jnt->col_id_ = jnt->active() ? (cid += jnt->dim()) - jnt->dim() : 0;
			}
			for (auto &mot:motionPool())
			{
				mot->col_id_ = mot->active() ? (cid += mot->dim()) - mot->dim() : 0;
			}

			imp->dyn_prt_dim_ = pid;
			imp->dyn_cst_dim_ = cid;
		}
		auto Model::dynUpd()->void
		{
			for (auto &prt : partPool())if (prt->active())prt->update();
			for (auto &jnt : jointPool())if (jnt->active())jnt->update();
			for (auto &mot : motionPool())if (mot->active())mot->update();
			for (auto &fce : forcePool())if (fce->active())fce->update();
		}
		auto Model::dynMtx(double *D, double *b) const->void
		{
			dynCstMtx(&D[(dynDim())*dynDimM()]);
			s_block_cpy(dynDimM(), dynDimN(), &D[(dynDim())*dynDimM()], 0, 0, dynDimN(), D, 0, dynDimM(), dynDim());
			
			dynIneMtx(&D[(dynDim())*dynDimM()]);
			s_block_cpy(dynDimM(), dynDimM(), -1, &D[(dynDim())*dynDimM()], 0, 0, dynDimM(), 0, D, 0, 0, dynDim());

			std::fill_n(&D[(dynDim())*dynDimM()], dynDimN()*(dynDim()), 0);
			s_block_cpyT(dynDimM(), dynDimN(), D, 0, dynDimM(), dynDim(), D, dynDimM(), 0, dynDim());

			dynPrtFce(b);
			dynCstAcc(b + dynDimM());
		}
		auto Model::dynSov(const double *D, const double *b, double *x) const->void
		{
			if (imp->dyn_solve_method_)
			{
				imp->dyn_solve_method_(dynDim(), D, b, x);
			}
			else
			{
				throw std::runtime_error("please set solve_method before use DynSov");
			}
		}
		auto Model::dynEnd(const double *x)->void
		{
			for (auto &prt : partPool())
			{
				if (prt->active())
				{
					std::copy_n(&x[prt->rowID()], 6, prt->imp->prt_acc_);
				}
			}
			for (auto &jnt : jointPool())
			{
				if (jnt->active())
				{
					std::copy_n(&x[jnt->col_id_ + dynDimM()], jnt->dim(), jnt->csf());
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot->active())
				{
					mot->mot_fce_dyn_ = x[mot->col_id_ + dynDimM()];
				}
			}
		}
		auto Model::dynUkn(double *x) const->void
		{
			this->dynPrtAcc(x);
			this->dynCstFce(x + dynDimM());
		}
		auto Model::dyn()->void
		{
			dynPre();
			std::vector<double> D(dynDim() * dynDim());
			std::vector<double> b(dynDim());
			std::vector<double> x(dynDim());
			dynUpd();
			dynMtx(D.data(), b.data());
			dynSov(D.data(), b.data(), x.data());
			dynEnd(x.data());
		}
		auto Model::clbSetInverseMethod(std::function<void(int n, double *A)> inverse_method)->void
		{
			imp->clb_inverse_method_ = inverse_method;
		}
		auto Model::clbDimM()const->std::size_t { return imp->clb_dim_m_; };
		auto Model::clbDimN()const->std::size_t { return imp->clb_dim_n_; };
		auto Model::clbDimGam()const->std::size_t { return imp->clb_dim_gam_; };
		auto Model::clbDimFrc()const->std::size_t { return imp->clb_dim_frc_; };
		auto Model::clbPre()->void
		{
			dynPre();

			if (dynDimN() != dynDimM())
			{
				throw std::runtime_error("must calibrate square matrix");
			}

			imp->clb_dim_m_ = 0;
			imp->clb_dim_n_ = 0;
			imp->clb_dim_gam_ = 0;
			imp->clb_dim_frc_ = 0;

			for (auto &i : motionPool())
			{
				if (i->active())
				{
					imp->clb_dim_m_++;
					imp->clb_dim_frc_ += 3;
					imp->clb_dim_n_ += 3;
				}
			}
			for (auto &i : partPool())
			{
				if (i->active())
				{
					imp->clb_dim_n_ += 10;
					imp->clb_dim_gam_ += 10;
				}
			}

		}
		auto Model::clbUpd()->void
		{
			dynUpd();
		}
		auto Model::clbMtx(double *clb_D, double *clb_b)const->void
		{
			if (!imp->clb_inverse_method_)throw std::runtime_error("please set inverse method before calibrate");
			if (dynDimN() != dynDimM()) throw std::logic_error("must calibrate square matrix");

			/*初始化*/
			Core::Matrix clb_d_m(clbDimM(), clbDimN());
			Core::Matrix clb_b_m(clbDimM(), 1);

			/*求A，即C的逆*/
			Core::Matrix A(dynDimM(), dynDimM()), B(dynDimM(), dynDimM());

			std::vector<double> C(dynDimM() * dynDimM());
			std::vector<double> f(dynDimM());

			dynCstMtx(C.data());
			std::copy(C.begin(), C.end(), A.data());
			imp->clb_inverse_method_(dynDimM(), A.data());

			/*求B*/
			const int beginRow = dynDimM() - clbDimM();

			for (auto &i:partPool())
			{
				if (i->active())
				{
					double cm[6][6];
					s_cmf(i->prtVel(), *cm);
					s_dgemm(clbDimM(), 6, 6, 1, &A(beginRow,i->rowID()), dynDimM(), *cm, 6, 0, &B(beginRow, i->rowID()), dynDimM());
				}
			}

			/*求解clb_d*/
			int col1 = 0, col2 = 0;

			for (auto &i:partPool())
			{
				if (i->active())
				{
					double q[6]{0};
					std::copy_n(i->prtAcc(), 6, q);
					s_daxpy(6, -1, i->prtGravity(), 1, q, 1);
					
					double v[6];
					std::copy_n(i->prtVel(), 6, v);

					for (std::size_t j = 0; j < clbDimM(); ++j)
					{
						clb_d_m(j, col1) = A(beginRow + j, col2 + 0) * q[0] + A(beginRow + j, col2 + 1) * q[1] + A(beginRow + j, col2 + 2) * q[2];
						clb_d_m(j, col1 + 1) = A(beginRow + j, col2 + 1) * q[5] + A(beginRow + j, col2 + 5) * q[1] - A(beginRow + j, col2 + 2) * q[4] - A(beginRow + j, col2 + 4) * q[2];
						clb_d_m(j, col1 + 2) = A(beginRow + j, col2 + 2) * q[3] + A(beginRow + j, col2 + 3) * q[2] - A(beginRow + j, col2 + 0) * q[5] - A(beginRow + j, col2 + 5) * q[0];
						clb_d_m(j, col1 + 3) = A(beginRow + j, col2 + 0) * q[4] + A(beginRow + j, col2 + 4) * q[0] - A(beginRow + j, col2 + 1) * q[3] - A(beginRow + j, col2 + 3) * q[1];
						clb_d_m(j, col1 + 4) = A(beginRow + j, col2 + 3) * q[3];
						clb_d_m(j, col1 + 5) = A(beginRow + j, col2 + 4) * q[4];
						clb_d_m(j, col1 + 6) = A(beginRow + j, col2 + 5) * q[5];
						clb_d_m(j, col1 + 7) = A(beginRow + j, col2 + 3) * q[4] + A(beginRow + j, col2 + 4) * q[3];
						clb_d_m(j, col1 + 8) = A(beginRow + j, col2 + 3) * q[5] + A(beginRow + j, col2 + 5) * q[3];
						clb_d_m(j, col1 + 9) = A(beginRow + j, col2 + 4) * q[5] + A(beginRow + j, col2 + 5) * q[4];

						clb_d_m(j, col1) += B(beginRow + j, col2 + 0) * v[0] + B(beginRow + j, col2 + 1) * v[1] + B(beginRow + j, col2 + 2) * v[2];
						clb_d_m(j, col1 + 1) += B(beginRow + j, col2 + 1) * v[5] + B(beginRow + j, col2 + 5) * v[1] - B(beginRow + j, col2 + 2) * v[4] - B(beginRow + j, col2 + 4) * v[2];
						clb_d_m(j, col1 + 2) += B(beginRow + j, col2 + 2) * v[3] + B(beginRow + j, col2 + 3) * v[2] - B(beginRow + j, col2 + 0) * v[5] - B(beginRow + j, col2 + 5) * v[0];
						clb_d_m(j, col1 + 3) += B(beginRow + j, col2 + 0) * v[4] + B(beginRow + j, col2 + 4) * v[0] - B(beginRow + j, col2 + 1) * v[3] - B(beginRow + j, col2 + 3) * v[1];
						clb_d_m(j, col1 + 4) += B(beginRow + j, col2 + 3) * v[3];
						clb_d_m(j, col1 + 5) += B(beginRow + j, col2 + 4) * v[4];
						clb_d_m(j, col1 + 6) += B(beginRow + j, col2 + 5) * v[5];
						clb_d_m(j, col1 + 7) += B(beginRow + j, col2 + 3) * v[4] + B(beginRow + j, col2 + 4) * v[3];
						clb_d_m(j, col1 + 8) += B(beginRow + j, col2 + 3) * v[5] + B(beginRow + j, col2 + 5) * v[3];
						clb_d_m(j, col1 + 9) += B(beginRow + j, col2 + 4) * v[5] + B(beginRow + j, col2 + 5) * v[4];
					}
					col1 += 10;
					col2 += 6;
				}
			}

			/*求解clb_b*/
			std::fill(f.begin(), f.end(), 0);
			int row = 0;
			for (auto &mot : motionPool())
			{
				if (mot->active())
				{
					clb_b_m(row, 0) = mot->mot_fce_;
					++row;
				}
			}
			for (auto &fce : forcePool())
			{
				if (fce->active())
				{
					s_daxpy(6, 1, fce->fceI(), 1, &f[fce->makI().fatherPart().rowID()], 1);
					s_daxpy(6, 1, fce->fceJ(), 1, &f[fce->makJ().fatherPart().rowID()], 1);
				}
			}
			s_dgemm(clbDimM(), 1, dynDimM(), 1, &A(beginRow,0), dynDimM(), f.data(), 1, 1, clb_b_m.data(), 1);

			/*以下添加驱动摩擦系数*/
			row = 0;
			for (auto &mot : motionPool())
			{
				//默认未激活的motion处于力控模式
				if (mot->active())
				{
					clb_d_m(row, clbDimGam() + row * 3) += s_sgn(mot->motVel());
					clb_d_m(row, clbDimGam() + row * 3 + 1) += mot->motVel();
					clb_d_m(row, clbDimGam() + row * 3 + 2) += mot->motAcc();
					++row;
				}
			}

			std::copy_n(clb_d_m.data(), clb_d_m.size(), clb_D);
			std::copy_n(clb_b_m.data(), clb_b_m.size(), clb_b);
		}
		auto Model::clbUkn(double *clb_x)const->void
		{
			int row = 0;
			for (auto &prt : partPool())
			{
				if (prt->active())
				{
					s_im2gamma(*prt->prtIm(), clb_x + row);
					row += 10;
				}
			}

			for (auto &mot : motionPool())
			{
				if (mot->active())
				{
					std::copy_n(mot->frcCoe(), 3, clb_x + row);
					row += 3;
				}
			}
		}
		auto Model::simKin(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval)->SimResult
		{
			//初始化变量
			SimResult result;
			result.resize(motionPool().size());
			std::list<double> time_akima_data;
			std::vector<std::list<double> > pos_akima_data(motionPool().size());

			//起始位置
			result.time_.push_back(0);
			time_akima_data.push_back(0);
			for (std::size_t i = 0; i < motionPool().size(); ++i)
			{
				motionPool().at(i).update();
				result.Pin_.at(i).push_back(motionPool().at(i).motPos());
				pos_akima_data.at(i).push_back(motionPool().at(i).motPos());
			}

			//其他位置
			for (param.count = 0; true; ++param.count)
			{
				auto is_sim = func(*this, param);

				result.time_.push_back(param.count + 1);
				for (std::size_t i = 0; i < motionPool().size(); ++i)
				{
					result.Pin_.at(i).push_back(motionPool().at(i).motPos());
				}

				if ((!is_sim) || ((param.count + 1) % akima_interval == 0))
				{
					time_akima_data.push_back((param.count + 1) / 1000.0);

					for (std::size_t j = 0; j < motionPool().size(); ++j)
					{
						pos_akima_data.at(j).push_back(motionPool().at(j).motPos());
					}
				}

				if (!is_sim)break;
			}

			//使用Akima储存电机位置数据
			for (std::size_t i = 0; i < motionPool().size(); ++i)
			{
				auto aki = akimaPool().find(motionPool().at(i).name() + "_akima");
				
				if (aki)
				{
					aki->operator=(Akima(aki->father(), aki->name(), aki->id(), time_akima_data, pos_akima_data.at(i)));
				}
				else
				{
					throw std::runtime_error("SimKin require motion akima element");
				}
			}

			return std::move(result);
		}
		auto Model::simDyn(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval, Script *script)->SimResult
		{
			saveDynEle("before_simDyn_state");
			auto result = simKin(func, param, akima_interval);
			loadDynEle("before_simDyn_state");

			result.Pin_.clear();
			result.Vin_.clear();
			result.Ain_.clear();
			result.Fin_.clear();

			result.Pin_.resize(motionPool().size());
			result.Vin_.resize(motionPool().size());
			result.Ain_.resize(motionPool().size());
			result.Fin_.resize(motionPool().size());

			//仿真计算
			for (std::size_t t = 0; t < result.time_.size(); ++t)
			{
				if (t % 100 == 0)std::cout << t << std::endl;

				if (script)script->doScript(t, t + 1);

				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					motionPool().at(j).mot_pos_ = akimaPool().at(j).operator()(t / 1000.0, '0');
				}
				kinFromPin();
				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					motionPool().at(j).mot_vel_ = akimaPool().at(j).operator()(t / 1000.0, '1');
				}
				kinFromVin();
				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					motionPool().at(j).mot_acc_ = akimaPool().at(j).operator()(t / 1000.0, '2');
				}
				dyn();
				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					result.Fin_.at(j).push_back(motionPool().at(j).mot_fce_dyn_);
					result.Pin_.at(j).push_back(motionPool().at(j).motPos());
					result.Vin_.at(j).push_back(motionPool().at(j).motVel());
					result.Ain_.at(j).push_back(motionPool().at(j).motAcc());
				}
			}

			return std::move(result);
		}
		auto Model::simToAdams(const std::string &filename, const PlanFunc &func, const PlanParamBase &param, int ms_dt, Script *script)->SimResult
		{
			saveDynEle("before_simToAdams_state");
			auto result = simDyn(func, param, ms_dt, script);
			loadDynEle("before_simToAdams_state");
			
			this->saveAdams(filename, true);
			return std::move(result);
		}

		RevoluteJoint::RevoluteJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ)
			: JointTemplate(father, name, id, makI, makJ) 
		{ 
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1,
				0,0,0,0,0
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}
		RevoluteJoint::RevoluteJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: JointTemplate(father, xml_ele, id) 
		{ 
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1,
				0,0,0,0,0
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}
		
		TranslationalJoint::TranslationalJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ)
			: JointTemplate(father, name, id, makI, makJ) 
		{ 
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}
		TranslationalJoint::TranslationalJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: JointTemplate(father, xml_ele, id) 
		{ 
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}

		UniversalJoint::UniversalJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ)
			: JointTemplate(father, name, id, makI, makJ) 
		{ 
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}
		UniversalJoint::UniversalJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: JointTemplate(father, xml_ele, id) 
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}
		auto UniversalJoint::saveAdams(std::ofstream &file) const->void
		{
			double pe[6] = { 0, 0, 0, PI / 2, 0, 0 };
			double pe2[6] = { 0, 0, 0, -PI / 2, 0, 0 };
			double pm[4][4], pm2[4][4];

			s_pe2pm(pe, *pm, "213");
			s_pm_dot_pm(*this->makI().prtPm(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << model().name() << "." << this->makI().fatherPart().name() << "." << this->makI().name() << " &\r\n"
				<< "    orientation = (" << Core::Matrix(1, 3, &pe[3]).toString() << ") \r\n"
				<< "!\r\n";

			s_pe2pm(pe2, *pm, "123");
			s_pm_dot_pm(*this->makJ().prtPm(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << model().name() << "." << this->makJ().fatherPart().name() << "." << this->makJ().name() << " &\r\n"
				<< "    orientation = (" << Core::Matrix(1, 3, &pe[3]).toString() << ") \r\n"
				<< "!\r\n";

			Constraint::saveAdams(file);
		}
		auto UniversalJoint::update()->void
		{
			/*update PrtCstMtxI*/
			makI().update();
			makJ().update();

			//get sin(a) and cos(a)
			double s = makI().pm()[0][2] * makJ().pm()[0][1]
				+ makI().pm()[1][2] * makJ().pm()[1][1]
				+ makI().pm()[2][2] * makJ().pm()[2][1];

			double c = makI().pm()[0][1] * makJ().pm()[0][1]
				+ makI().pm()[1][1] * makJ().pm()[1][1]
				+ makI().pm()[2][1] * makJ().pm()[2][1];

			csmI_[3][3] = -(makI().prtPm()[0][1]) * s + (makI().prtPm()[0][2]) * c;
			csmI_[4][3] = -(makI().prtPm()[1][1]) * s + (makI().prtPm()[1][2]) * c;
			csmI_[5][3] = -(makI().prtPm()[2][1]) * s + (makI().prtPm()[2][2]) * c;

			/*edit CstMtxJ*/
			std::fill_n(static_cast<double *>(csmJ()), this->dim() * 6, 0);
			double pm_M2N[4][4];
			s_pm_dot_pm(*makJ().fatherPart().invPm(), *makI().fatherPart().pm(), *pm_M2N);
			s_tf_n(Dim(), -1, *pm_M2N, csmI(), 0, csmJ());



			/*update A_c*/
			std::fill_n(csa(), UniversalJoint::dim(), 0);

			/*calculate a_dot*/
			double v[3];
			v[0] = makJ().vel()[3] - makI().vel()[3];
			v[1] = makJ().vel()[4] - makI().vel()[4];
			v[2] = makJ().vel()[5] - makI().vel()[5];

			double a_dot = makI().pm()[0][0] * v[0] + makI().pm()[1][0] * v[1] + makI().pm()[2][0] * v[2];

			/*calculate part m*/
			v[0] = -c*a_dot;
			v[1] = -s*a_dot;

			double tem_v1[6]{ 0 }, tem_v2[6]{ 0 };
			s_inv_tv(*makI().prtPm(), makI().fatherPart().prtVel(), tem_v1);
			csa()[3] -= v[0] * tem_v1[4] + v[1] * tem_v1[5];

			/*calculate part n*/
			s_inv_tv(*pm_M2N, makJ().fatherPart().prtVel(), tem_v1);
			s_cv(-1, makI().fatherPart().prtVel(), tem_v1, 0, tem_v2);
			s_dgemmTN(4, 1, 6, 1, csmI(), Dim(), tem_v2, 1, 1, &csa()[0], 1);
			s_inv_tv(*makI().prtPm(), tem_v1, tem_v2);
			csa()[3] += v[0] * tem_v2[4] + v[1] * tem_v2[5];
		};

		SphericalJoint::SphericalJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ)
			: JointTemplate(father, name, id, makI, makJ) 
		{ 
			const static double loc_cst[6][Dim()]
			{
				1,0,0,
				0,1,0,
				0,0,1,
				0,0,0,
				0,0,0,
				0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}
		SphericalJoint::SphericalJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: JointTemplate(father, xml_ele, id) 
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,
				0,1,0,
				0,0,1,
				0,0,0,
				0,0,0,
				0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, csmI());
		}

		SingleComponentMotion::SingleComponentMotion(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, int component_axis)
			: Motion(father, name, id, makI, makJ), component_axis_(component_axis)
		{
			double loc_cst[6]{ 0,0,0,0,0,0, };
			loc_cst[component_axis_] = 1;
			s_tf(*this->makI().prtPm(), loc_cst, csmI());
		}
		SingleComponentMotion::SingleComponentMotion(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: Motion(father, xml_ele, id), component_axis_(std::stoi(xml_ele.Attribute("component")))
		{
			double loc_cst[6]{ 0,0,0,0,0,0, };
			loc_cst[component_axis_] = 1;
			s_tf(*this->makI().prtPm(), loc_cst, csmI());
		}
		auto SingleComponentMotion::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Motion::saveXml(xml_ele);
			xml_ele.SetAttribute("component", this->component_axis_);
		}
		auto SingleComponentMotion::saveAdams(std::ofstream &file) const->void
		{
			std::string s;

			switch (component_axis_)
			{
			case 0:
				s = "x";
				break;
			case 1:
				s = "y";
				break;
			case 2:
				s = "z";
				break;
			case 3:
				s = "B1";
				break;
			case 4:
				s = "B2";
				break;
			case 5:
				s = "B3";
				break;
			}

			if (false)
			{
				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    adams_id = " << adamsID() << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << this->makI().fatherPart().name() << "." << this->makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << this->makJ().fatherPart().name() << "." << this->makJ().name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"" << this->motPos() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    adams_id = " << adamsID() << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << this->makI().fatherPart().name() << "." << this->makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << this->makJ().fatherPart().name() << "." << this->makJ().name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"AKISPL(time,0," << this->name() << "_akima)\"  \r\n"
					<< "!\r\n";
			}
		}
		auto SingleComponentMotion::update()->void
		{
			/*update motPos motVel,  motAcc should be given, not computed by part acc*/
			makI().update();
			makJ().update();

			double pm_I2J[4][4], pe[6];
			s_inv_pm_dot_pm(*makJ().pm(), *makI().pm(), *pm_I2J);
			s_pm2pe(&pm_I2J[0][0], pe, "123");
			mot_pos_ = pe[component_axis_];

			double velDiff[6], velDiff_in_J[6];
			std::copy_n(makI().vel(), 6, velDiff);
			s_daxpy(6, -1, makJ().vel(), 1, velDiff, 1);
			s_inv_tv(*makJ().pm(), velDiff, velDiff_in_J);
			mot_vel_ = velDiff_in_J[component_axis_];

			/*update cst mtx*/
			std::fill_n(csmJ(), 6, 0);
			double pm_M2N[4][4];
			s_pm_dot_pm(*makJ().fatherPart().invPm(), *makI().fatherPart().pm(), *pm_M2N);
			s_tf(-1, *pm_M2N, csmI(), 0, csmJ());

			/*update a_c*/
			std::fill_n(csa(), 1, 0);
			double tem_v1[6]{ 0 }, tem_v2[6]{ 0 };
			s_inv_tv(-1, *pm_M2N, makJ().fatherPart().prtVel(), 0, tem_v1);
			s_cv(makI().fatherPart().prtVel(), tem_v1, tem_v2);
			s_dgemmTN(1, 1, 6, 1, csmI(), 1, tem_v2, 1, 0, csa(), 1);

			csa()[0] += mot_acc_;
			/*update motPos motVel motAcc*/
		}
		
		SingleComponentForce::SingleComponentForce(Object &father, const std::string &name, std::size_t id, Marker& makI, Marker& makJ, int componentID)
			: Force(father, name, id, makI, makJ), component_axis_(componentID)
		{

		}
		SingleComponentForce::SingleComponentForce(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
			: Force(father, xml_ele, id), component_axis_(std::stoi(xml_ele.Attribute("component")))
		{
		}
		auto SingleComponentForce::saveXml(Aris::Core::XmlElement &xml_ele) const->void
		{
			Force::saveXml(xml_ele);
			xml_ele.SetAttribute("component", this->component_axis_);
		}
		auto SingleComponentForce::saveAdams(std::ofstream &file) const->void
		{
			if (true)
			{
				std::string type = "translational";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << model().name() << "." << name() << "  &\r\n"
					<< "    adams_id = " << adamsID() << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << makI().fatherPart().name() << "." << makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << makJ().fatherPart().name() << "." << makJ().name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"" << fce() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				std::string type = "translational";

				

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << model().name() << "." << name() << "  &\r\n"
					<< "    adams_id = " << adamsID() << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << makI().fatherPart().name() << "." << makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << makJ().fatherPart().name() << "." << makJ().name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"AKISPL(time,0," << name() << "_fce_spl)\"  \r\n"
					<< "!\r\n";
			}
		}
		auto SingleComponentForce::update()->void
		{
			s_tf(*makI().prtPm(), fce_value_, fceI_);
			double pm_M2N[16];
			s_inv_pm_dot_pm(*makJ().fatherPart().pm(), *makI().fatherPart().pm(), pm_M2N);
			s_tf(-1, pm_M2N, fceI_, 0, fceJ_);
		}
	}
}
